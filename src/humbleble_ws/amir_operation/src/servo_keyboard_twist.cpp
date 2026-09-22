// MoveIt Servo キーボード操作ノード (速度保持方式)
//
// キーボードで手先 Twist を生成し /servo_node/delta_twist_cmds へ publish する。
// 「押すたびにその軸の速度をセットし、別の指示があるまで保持し続ける」方式。
// ターミナルのキーリピート(初回待ち)に依存しないため確実に連続移動できる。
//
// キー割り当て (robot_link_command_frame = base_footprint 基準):
//   w / s : +X / -X  (前後)
//   a / d : +Y / -Y  (左右)
//   r / f : +Z / -Z  (上下)
//   space : 停止 (全軸ゼロ)
//   [ / ] : 速度を下げる / 上げる
//   q     : 終了
//
// 各軸は独立に保持されるので、w の後に a を押すと斜め移動になる。
// 単一方向に戻したいときは space で一度ゼロにしてから押し直す。
//
// 事前に必要 (Servo 出力先を有効化):
//   ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
//
// 実行:
//   ros2 run amir_operation servo_keyboard_twist
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <chrono>
#include <algorithm>

using namespace std::chrono_literals;

// 端末を raw / 非ブロッキングにし、デストラクタで元へ戻す RAII ヘルパ
class TerminalRawMode
{
public:
  TerminalRawMode()
  {
    tcgetattr(STDIN_FILENO, &orig_);
    termios raw = orig_;
    raw.c_lflag &= ~(ICANON | ECHO);  // 行バッファ・エコーを無効化
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

class ServoKeyboardTwist : public rclcpp::Node
{
public:
  explicit ServoKeyboardTwist(const rclcpp::NodeOptions & options)
  : Node("servo_keyboard_twist", options)
  {
    lin_speed_  = this->declare_parameter<double>("linear_speed", 0.08);     // [m/s]
    z_speed_    = this->declare_parameter<double>("z_speed", 0.08);          // [m/s]
    speed_step_ = this->declare_parameter<double>("speed_step", 0.02);       // [m/s]
    frame_      = this->declare_parameter<std::string>("command_frame", "base_footprint");

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

    start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    start_timer_ = this->create_wall_timer(
      500ms, std::bind(&ServoKeyboardTwist::try_start_servo, this));
  }

private:
  void try_start_servo()
  {
    if (!start_client_->wait_for_service(0s)) {
      RCLCPP_INFO(get_logger(), "⏳ /servo_node/start_servo 待機中...");
      return;
    }
    start_timer_->cancel();
    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    start_client_->async_send_request(
      req, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
        RCLCPP_INFO(get_logger(), "✅ start_servo: %s",
                    future.get()->success ? "成功" : "失敗");
        print_help();
        // 100 Hz でキー読み取り＋publish
        loop_timer_ = create_wall_timer(
          10ms, std::bind(&ServoKeyboardTwist::loop, this));
      });
  }

  void print_help()
  {
    RCLCPP_INFO(get_logger(),
      "\n==== Servo キーボード操作 (速度保持方式 / frame=%s) ====\n"
      "  w/s : +X/-X (前後)   a/d : +Y/-Y (左右)   r/f : +Z/-Z (上下)\n"
      "  space : 停止(全軸ゼロ)   [ / ] : 速度 -/+   q : 終了\n"
      "  ※押した方向に保持して動き続けます。止めたいときは space。\n"
      "  現在速度 lin=%.3f m/s, z=%.3f m/s\n"
      "=========================================",
      frame_.c_str(), lin_speed_, z_speed_);
  }

  void loop()
  {
    // たまった入力をすべて処理する (保持方式なので最新で上書きされる)
    char c;
    while (read(STDIN_FILENO, &c, 1) == 1) {
      switch (c) {
        case 'w': vx_ =  lin_speed_; break;
        case 's': vx_ = -lin_speed_; break;
        case 'a': vy_ =  lin_speed_; break;
        case 'd': vy_ = -lin_speed_; break;
        case 'r': vz_ =  z_speed_;   break;
        case 'f': vz_ = -z_speed_;   break;
        case ' ': vx_ = vy_ = vz_ = 0.0; break;  // 停止
        case '[':
          lin_speed_ = std::max(0.0, lin_speed_ - speed_step_);
          z_speed_   = std::max(0.0, z_speed_   - speed_step_);
          rescale();
          RCLCPP_INFO(get_logger(), "速度 lin=%.3f z=%.3f", lin_speed_, z_speed_);
          break;
        case ']':
          lin_speed_ += speed_step_;
          z_speed_   += speed_step_;
          rescale();
          RCLCPP_INFO(get_logger(), "速度 lin=%.3f z=%.3f", lin_speed_, z_speed_);
          break;
        case 'q':
          RCLCPP_INFO(get_logger(), "終了します");
          rclcpp::shutdown();
          return;
        default:
          break;  // 未割り当てキーは無視 (保持中の速度は変えない)
      }
    }

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_;
    msg.twist.linear.x = vx_;
    msg.twist.linear.y = vy_;
    msg.twist.linear.z = vz_;
    twist_pub_->publish(msg);
  }

  // 速度変更時、保持中の各軸の符号を保ったまま大きさを更新する
  void rescale()
  {
    if (vx_ != 0.0) vx_ = (vx_ > 0 ? 1 : -1) * lin_speed_;
    if (vy_ != 0.0) vy_ = (vy_ > 0 ? 1 : -1) * lin_speed_;
    if (vz_ != 0.0) vz_ = (vz_ > 0 ? 1 : -1) * z_speed_;
  }

  double lin_speed_, z_speed_, speed_step_;
  std::string frame_;
  double vx_{0.0}, vy_{0.0}, vz_{0.0};
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr loop_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  TerminalRawMode raw_mode;  // main の間だけ端末を raw に (スコープ終了で必ず復元)
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<ServoKeyboardTwist>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
