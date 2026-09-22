#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

class GzPoseFilter : public rclcpp::Node
{
public:
  GzPoseFilter() : Node("gz_pose_filter")
  {
    declare_parameter("world_name", "warehouse_world");
    declare_parameter("robot_name", "amir_mecanum3");
    declare_parameter("object_prefix", "target_obj");

    world_name_    = get_parameter("world_name").as_string();
    robot_name_    = get_parameter("robot_name").as_string();
    object_prefix_ = get_parameter("object_prefix").as_string();

    std::string topic = "/world/" + world_name_ + "/pose/info";

    sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
      topic, 10,
      std::bind(&GzPoseFilter::cb, this, std::placeholders::_1));

    pub_ = create_publisher<tf2_msgs::msg::TFMessage>("/gazebo/model_poses", 10);

    RCLCPP_INFO(get_logger(), "Filtering: %s  (robot=%s, prefix=%s)",
      topic.c_str(), robot_name_.c_str(), object_prefix_.c_str());
  }

private:
  void cb(const tf2_msgs::msg::TFMessage::SharedPtr msg)
  {
    tf2_msgs::msg::TFMessage out;
    for (const auto & tf : msg->transforms) {
      const auto & name = tf.child_frame_id;
      if (name == robot_name_ || name.rfind(object_prefix_, 0) == 0) {
        out.transforms.push_back(tf);
      }
    }
    if (!out.transforms.empty()) {
      pub_->publish(out);
    }
  }

  std::string world_name_;
  std::string robot_name_;
  std::string object_prefix_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr sub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GzPoseFilter>());
  rclcpp::shutdown();
  return 0;
}
