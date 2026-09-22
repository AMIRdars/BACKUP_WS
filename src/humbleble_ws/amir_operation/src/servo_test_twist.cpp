// MoveIt Servo 動作確認用テストノード
//
// /servo_node/start_servo を呼んだ後、手先 Twist を緩やかに振動させて
// /servo_node/delta_twist_cmds に publish する。
// VR ブリッジ無しで Servo → forward_position_controller → ign-gazebo の
// 経路が動くかを確認するためのもの。
//
// 事前に必要 (Servo出力先を有効化):
//   ros2 control switch_controllers --deactivate arm_controller --activate forward_position_controller
//
// 実行:
//   ros2 run amir_operation servo_test_twist
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class ServoTestTwist : public rclcpp::Node
{
public:
  explicit ServoTestTwist(const rclcpp::NodeOptions & options)
  : Node("servo_test_twist", options)
  {
    // 振幅・周期はパラメータで調整可能
    lin_amp_ = this->declare_parameter<double>("linear_amplitude", 0.05);   // [m/s]
    ang_amp_ = this->declare_parameter<double>("angular_amplitude", 0.0);   // [rad/s]
    period_  = this->declare_parameter<double>("period", 6.0);             // [s]
    frame_   = this->declare_parameter<std::string>("command_frame", "base_footprint");

    twist_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
      "/servo_node/delta_twist_cmds", rclcpp::SystemDefaultsQoS());

    start_client_ = this->create_client<std_srvs::srv::Trigger>("/servo_node/start_servo");

    // 起動直後に Servo を開始してから publish を始める
    start_timer_ = this->create_wall_timer(
      500ms, std::bind(&ServoTestTwist::try_start_servo, this));
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
        t0_ = now();
        pub_timer_ = create_wall_timer(
          10ms, std::bind(&ServoTestTwist::publish_twist, this));  // 100 Hz
      });
  }

  void publish_twist()
  {
    const double t = (now() - t0_).seconds();
    const double w = 2.0 * M_PI / period_;

    geometry_msgs::msg::TwistStamped msg;
    msg.header.stamp = now();
    msg.header.frame_id = frame_;
    // X 並進を正弦波で往復。必要なら angular も。
    msg.twist.linear.x  = lin_amp_ * std::sin(w * t);
    msg.twist.angular.z = ang_amp_ * std::sin(w * t);
    twist_pub_->publish(msg);
  }

  double lin_amp_, ang_amp_, period_;
  std::string frame_;
  rclcpp::Time t0_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_client_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  options.parameter_overrides({{"use_sim_time", true}});
  auto node = std::make_shared<ServoTestTwist>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
