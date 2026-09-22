#include <rclcpp/rclcpp.hpp>
#include <amir_interfaces/msg/amir_cmd.hpp>
#include <string>
#include <vector>

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("vis_joint_publisher")
  {
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();
    joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", qos_profile);

    // 初期角度と速度（angle[0] = -2967.0 を起点）
    angle_ = {-2967.0, -740.0, 1000.0, -1700.0, 2748.0, -1250.0};
    vel_   = {400.0, 200.0, 400.0, 400.0, 400.0, 200.0};

    publish_swing();  // 1回だけスイング動作を行う
  }

private:
  void publish_joint_states()
  {
    auto joint = amir_interfaces::msg::AmirCmd();
    for (int i = 0; i < 6; ++i) {
      joint.angle[i] = angle_[i];
      joint.vel[i] = vel_[i];
    }

    for (int i = 0; i < 5; ++i) {
      joint_pub_->publish(joint);
      rclcpp::sleep_for(std::chrono::milliseconds(20));
    }

    RCLCPP_INFO(this->get_logger(), "📡 送信中 angle[0] = %.1f", angle_[0]);
  }

  void publish_swing()
  {
    rclcpp::Rate rate(50);  // 20ms周期

    const float max_pos = 0.0;       // 右端
    const float min_pos = -5934.0;   // 左端
    float direction = 1.0;

    bool forward_done = false;
    bool backward_done = false;

    while (rclcpp::ok()) {
      if (!forward_done) {
        angle_[0] += direction * vel_[0] * 0.05;
        if (angle_[0] >= max_pos) {
          angle_[0] = max_pos;
          direction = -1.0;
          forward_done = true;
          std::cout << "➡️ 最大位置 0 に到達\n";
        }
      } else if (!backward_done) {
        angle_[0] += direction * vel_[0] * 0.05;
        if (angle_[0] <= min_pos) {
          angle_[0] = min_pos;
          backward_done = true;
          std::cout << "⬅️ 最小位置 -5934 に到達\n";
        }
      }

      publish_joint_states();
      rate.sleep();

      if (forward_done && backward_done) break;
    }

    std::cout << "✅ 1往復完了（0 ➡️ -5934）\n";
  }

  rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
  std::vector<float> angle_;
  std::vector<float> vel_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPublisher>();
  rclcpp::spin_some(node);
  rclcpp::shutdown();
  return 0;
}
