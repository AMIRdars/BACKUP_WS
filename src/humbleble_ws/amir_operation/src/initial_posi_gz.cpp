#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/gripper_command.hpp>
#include <chrono>

using namespace std::chrono_literals;
using GripperCommand = control_msgs::action::GripperCommand;

class JointTrajectoryPublisher : public rclcpp::Node
{
public:
  JointTrajectoryPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("vis_joint_publisher", options)
  {
    // 相対名: ノードの namespace で /<ns>/arm_controller, /<ns>/gripper_controller に解決
    arm_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      "arm_controller/joint_trajectory", 10);

    gripper_client_ = rclcpp_action::create_client<GripperCommand>(
      this, "gripper_controller/gripper_cmd");

    // 修正：create_timer を create_wall_timer に戻す
    timer_ = this->create_wall_timer(
      1000ms, std::bind(&JointTrajectoryPublisher::publish_joint_states, this));
  }

private:
  void publish_joint_states()
  {
    auto arm_msg = trajectory_msgs::msg::JointTrajectory();
    
    // 【維持】現在のシミュレーション時刻をヘッダにセット（これが最も重要）
    arm_msg.header.stamp = this->now();
    arm_msg.joint_names = {"Joint_1", "Joint_2", "Joint_3", "Joint_4", "Joint_5"};

    trajectory_msgs::msg::JointTrajectoryPoint arm_point;
    // arm_point.positions  = {0.00, 2.3, -2.3, -0.50, 0};
    arm_point.positions  = {0.00, 2.3, -2.3, -0.2, 0};
    arm_point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0};
    arm_point.time_from_start.sec     = 5;
    arm_point.time_from_start.nanosec = 0;
    
    arm_msg.points.push_back(arm_point);
    arm_pub_->publish(arm_msg);
    RCLCPP_INFO(this->get_logger(), "🚀 Published arm pos1 to /arm_controller.");

    if (!gripper_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(this->get_logger(), "❌ gripper_cmd action server not available!");
      timer_->cancel();
      return;
    }

    auto goal = GripperCommand::Goal();
    goal.command.position   = -0.8;
    goal.command.max_effort = 10.0;

    gripper_client_->async_send_goal(goal);
    RCLCPP_INFO(this->get_logger(), "🤏 Sent gripper goal: %.2f rad", goal.command.position);

    timer_->cancel();
  }

  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr arm_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions options;
  options.parameter_overrides({
    {"use_sim_time", true}
  });

  auto node = std::make_shared<JointTrajectoryPublisher>(options);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}