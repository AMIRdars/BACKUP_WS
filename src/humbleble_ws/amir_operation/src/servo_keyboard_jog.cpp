// MoveIt Servo キーボード操作ノード (3DOF位置ジョグ / MoveItヤコビアン版)
//
// 5軸アームを「並進3自由度だけ」素直に動かす版。
// Servo の Cartesian(twist) 経路は 6x5 ヤコビアンを使い angular=0 が拘束になって
// 過剰拘束になるため、本ノードが自前で 3x5 の位置ヤコビアンだけで微分IK
//   dq = J_pos^+ * v   (減衰最小二乗)
// を解き、関節速度を Servo の関節ジョグ入力 /servo_node/delta_joint_cmds
// (control_msgs/JointJog) へ渡す。Servo は平滑化・関節限界・衝突回避のみ担当。
//
// ヤコビアンは MoveIt の RobotState::getJacobian で計算する。これは Servo 本体が
// 使うのと同じもので、planning フレーム(= モデルのルート link = base_footprint)で
// 返ると保証されているため、フレームずれの定数補正は不要(機体/URDFに依存しない)。
//
// URDF と SRDF は既定で ament の share から読む(引数なしで ros2 run 可能)。
// 別機体なら urdf_path / srdf_path / group / tip_link を上書きする。
//
// キー割り当て (base_footprint 基準, 速度保持方式):
//   w/s : +X/-X   a/d : +Y/-Y   r/f : +Z/-Z
//   space : 停止   [ / ] : 速度 -/+   q : 終了
//
// 事前に必要 (Servo 出力先を有効化):
//   ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <control_msgs/msg/joint_jog.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <srdfdom/model.h>
#include <urdf_parser/urdf_parser.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <Eigen/Dense>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <sstream>
#include <chrono>
#include <algorithm>
#include <map>

using namespace std::chrono_literals;

class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);
    orig_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, orig_flags_ | O_NONBLOCK);
  }
  ~TerminalRawMode()
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_);
    fcntl(STDIN_FILENO, F_SETFL, orig_flags_);
  }
private:
  termios orig_{};
  int orig_flags_{0};
};

class ServoKeyboardJog : public rclcpp::Node
{
public:
  explicit ServoKeyboardJog(const rclcpp::NodeOptions & options)
  : Node("servo_keyboard_jog", options)
  {
    lin_speed_  = this->declare_parameter<double>("linear_speed", 0.15);
    z_speed_    = this->declare_parameter<double>("z_speed", 0.08);
    speed_step_ = this->declare_parameter<double>("speed_step", 0.02);
    damping_    = this->declare_parameter<double>("damping", 0.05);
    group_name_ = this->declare_parameter<std::string>("group", "arm");
    tip_link_   = this->declare_parameter<std::string>("tip_link", "tcp_link");

    // 既定で sim と同じ moveit_config の xacro を展開して使う(取り付け向きまで一致)。
    // 古い amir_description の plain urdf は base 取付けが 90° 違うので使わない。
    std::string def_xacro, def_srdf;
    try {
      def_xacro = ament_index_cpp::get_package_share_directory("amir_moveit_config")
                  + "/config/amir_mecanum3.urdf.xacro";
      def_srdf  = ament_index_cpp::get_package_share_directory("amir_moveit_config")
                  + "/config/amir_mecanum3.srdf";
    } catch (...) {}
    auto xacro_path = this->declare_parameter<std::string>("xacro_path", def_xacro);
    auto urdf_path  = this->declare_parameter<std::string>("urdf_path", "");   // 展開済みURDFを直接使う場合
    auto srdf_path  = this->declare_parameter<std::string>("srdf_path", def_srdf);
    // launch から直接 URDF/SRDF 文字列を渡すこともできる(最優先)
    auto urdf_xml   = this->declare_parameter<std::string>("robot_description", "");
    auto srdf_xml   = this->declare_parameter<std::string>("robot_description_semantic", "");

    if (!build_model(urdf_xml, srdf_xml, xacro_path, urdf_path, srdf_path)) {
      throw std::runtime_error("RobotModel の構築に失敗しました");
    }

    jog_pub_ = this->create_publisher<control_msgs::msg::JointJog>(
      "/servo_node/delta_joint_cmds", rclcpp::SystemDefaultsQoS());
    js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::JointState::SharedPtr m) {
        for (size_t i = 0; i < m->name.size(); ++i) pos_[m->name[i]] = m->position[i];
      });
    start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    start_timer_ = this->create_wall_timer(
      500ms, std::bind(&ServoKeyboardJog::try_start_servo, this));
  }

private:
  static std::string read_file(const std::string & path)
  {
    std::ifstream f(path);
    if (!f) return "";
    std::stringstream ss; ss << f.rdbuf(); return ss.str();
  }

  // xacro CLI を実行して展開済み URDF を得る
  static std::string run_xacro(const std::string & path)
  {
    std::string out;
    FILE * p = popen(("xacro '" + path + "' 2>/dev/null").c_str(), "r");
    if (!p) return "";
    char buf[8192]; size_t n;
    while ((n = fread(buf, 1, sizeof(buf), p)) > 0) out.append(buf, n);
    pclose(p);
    return out;
  }

  bool build_model(std::string urdf_xml, std::string srdf_xml,
                   const std::string & xacro_path, const std::string & urdf_path,
                   const std::string & srdf_path)
  {
    // URDF 取得の優先順: 文字列パラメータ > xacro展開 > 展開済みファイル
    if (urdf_xml.empty() && !xacro_path.empty()) urdf_xml = run_xacro(xacro_path);
    if (urdf_xml.empty() && !urdf_path.empty())  urdf_xml = read_file(urdf_path);
    if (srdf_xml.empty()) srdf_xml = read_file(srdf_path);
    if (urdf_xml.empty() || srdf_xml.empty()) {
      RCLCPP_FATAL(get_logger(), "URDF(xacro=%s urdf=%s) または SRDF(%s) を読めません。",
                   xacro_path.c_str(), urdf_path.c_str(), srdf_path.c_str());
      return false;
    }
    auto urdf_model = urdf::parseURDF(urdf_xml);
    if (!urdf_model) { RCLCPP_FATAL(get_logger(), "URDF パース失敗"); return false; }
    auto srdf_model = std::make_shared<srdf::Model>();
    if (!srdf_model->initString(*urdf_model, srdf_xml)) {
      RCLCPP_FATAL(get_logger(), "SRDF パース失敗"); return false;
    }
    robot_model_ = std::make_shared<moveit::core::RobotModel>(urdf_model, srdf_model);
    jmg_ = robot_model_->getJointModelGroup(group_name_);
    tip_ = robot_model_->getLinkModel(tip_link_);
    if (!jmg_ || !tip_) {
      RCLCPP_FATAL(get_logger(), "group '%s' または tip '%s' が見つかりません",
                   group_name_.c_str(), tip_link_.c_str());
      return false;
    }
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);
    robot_state_->setToDefaultValues();
    joint_names_ = jmg_->getActiveJointModelNames();
    std::string js; for (auto & s : joint_names_) js += s + " ";
    RCLCPP_INFO(get_logger(), "RobotModel OK  modelフレーム=%s  group=%s(%zu軸)  tip=%s  [%s]",
                robot_model_->getModelFrame().c_str(), group_name_.c_str(),
                joint_names_.size(), tip_link_.c_str(), js.c_str());
    return joint_names_.size() >= 3;
  }

  void try_start_servo()
  {
    if (!start_client_->wait_for_service(0s)) {
      RCLCPP_INFO(get_logger(), "⏳ /servo_node/start_servo 待機中...");
      return;
    }
    start_timer_->cancel();
    start_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>(),
      [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture f) {
        RCLCPP_INFO(get_logger(), "✅ start_servo: %s", f.get()->success ? "成功" : "失敗");
        print_help();
        loop_timer_ = create_wall_timer(10ms, std::bind(&ServoKeyboardJog::loop, this));
      });
  }

  void print_help()
  {
    RCLCPP_INFO(get_logger(),
      "\n==== Servo キーボード操作 (3DOF位置ジョグ / frame=%s) ====\n"
      "  w/s : +X/-X   a/d : +Y/-Y   r/f : +Z/-Z\n"
      "  space : 停止   [ / ] : 速度 -/+   q : 終了\n"
      "  ※MoveItヤコビアンで planning フレーム基準。角速度拘束なし。\n"
      "  現在速度 lin=%.3f, z=%.3f m/s\n"
      "=========================================",
      robot_model_->getModelFrame().c_str(), lin_speed_, z_speed_);
  }

  void read_keys()
  {
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      switch (c) {
        case 'w': vx_ =  lin_speed_; break;
        case 's': vx_ = -lin_speed_; break;
        case 'a': vy_ =  lin_speed_; break;
        case 'd': vy_ = -lin_speed_; break;
        case 'r': vz_ =  z_speed_;   break;
        case 'f': vz_ = -z_speed_;   break;
        case ' ': vx_ = vy_ = vz_ = 0.0; break;
        case '[':
          lin_speed_ = std::max(0.0, lin_speed_ - speed_step_);
          z_speed_   = std::max(0.0, z_speed_   - speed_step_); rescale();
          RCLCPP_INFO(get_logger(), "速度 lin=%.3f z=%.3f", lin_speed_, z_speed_); break;
        case ']':
          lin_speed_ += speed_step_; z_speed_ += speed_step_; rescale();
          RCLCPP_INFO(get_logger(), "速度 lin=%.3f z=%.3f", lin_speed_, z_speed_); break;
        case 'q':
          RCLCPP_INFO(get_logger(), "終了します"); rclcpp::shutdown(); return;
        default: break;
      }
    }
  }

  void rescale()
  {
    if (vx_ != 0.0) vx_ = (vx_ > 0 ? 1 : -1) * lin_speed_;
    if (vy_ != 0.0) vy_ = (vy_ > 0 ? 1 : -1) * lin_speed_;
    if (vz_ != 0.0) vz_ = (vz_ > 0 ? 1 : -1) * z_speed_;
  }

  void loop()
  {
    read_keys();

    // 現在の関節角を group に反映 (未受信なら送らない)
    std::vector<double> q(joint_names_.size());
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      auto it = pos_.find(joint_names_[i]);
      if (it == pos_.end()) return;
      q[i] = it->second;
    }
    robot_state_->setJointGroupPositions(jmg_, q);
    robot_state_->updateLinkTransforms();

    // MoveIt ヤコビアン (6 x n, planning フレーム基準) の並進3行で減衰最小二乗
    Eigen::MatrixXd J;
    if (!robot_state_->getJacobian(jmg_, tip_, Eigen::Vector3d::Zero(), J)) return;
    Eigen::MatrixXd Jp = J.topRows(3);                          // 3 x n
    Eigen::Vector3d v(vx_, vy_, vz_);                           // planning フレーム
    Eigen::Matrix3d M = Jp * Jp.transpose()
                        + damping_ * damping_ * Eigen::Matrix3d::Identity();
    Eigen::VectorXd dq = Jp.transpose() * M.ldlt().solve(v);    // [rad/s]

    control_msgs::msg::JointJog msg;
    msg.header.stamp = now();
    msg.joint_names = joint_names_;
    msg.velocities.assign(dq.data(), dq.data() + dq.size());
    jog_pub_->publish(msg);
  }

  double lin_speed_, z_speed_, speed_step_, damping_;
  std::string group_name_, tip_link_;
  double vx_{0.0}, vy_{0.0}, vz_{0.0};

  moveit::core::RobotModelPtr robot_model_;
  moveit::core::RobotStatePtr robot_state_;
  const moveit::core::JointModelGroup * jmg_{nullptr};
  const moveit::core::LinkModel * tip_{nullptr};
  std::vector<std::string> joint_names_;
  std::map<std::string, double> pos_;

  rclcpp::Publisher<control_msgs::msg::JointJog>::SharedPtr jog_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr js_sub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TerminalRawMode raw_mode;
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<ServoKeyboardJog>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
