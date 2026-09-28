#include <algorithm>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

// Extract model transforms in C++ before Python deserialization. Values,
// frame IDs and timestamps are copied exactly; robot models precede payload.
class TransportPoseFilter : public rclcpp::Node {
 public:
  TransportPoseFilter() : Node("transport_pose_filter") {
    const auto input=declare_parameter<std::string>("input_topic","/world/cooperative_transport_friction/pose/info");
    const auto output=declare_parameter<std::string>("output_topic","/cooperative_transport/model_poses");
    names_=declare_parameter<std::vector<std::string>>("entity_names",{"amir1","amir2","cooperative_payload"});
    publisher_=create_publisher<tf2_msgs::msg::TFMessage>(output,10);
    subscription_=create_subscription<tf2_msgs::msg::TFMessage>(input,10,
      [this](const tf2_msgs::msg::TFMessage::SharedPtr message) {
        tf2_msgs::msg::TFMessage result;
        result.transforms.reserve(names_.size());
        for(const auto &name:names_) {
          auto found=std::find_if(message->transforms.begin(),message->transforms.end(),
            [&name](const auto &transform){return transform.child_frame_id==name;});
          if(found!=message->transforms.end())result.transforms.push_back(*found);
        }
        if(!result.transforms.empty())publisher_->publish(result);
      });
    RCLCPP_INFO(get_logger(),"Filtering %s -> %s (%zu models)",input.c_str(),output.c_str(),names_.size());
  }
 private:
  std::vector<std::string> names_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr publisher_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr subscription_;
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<TransportPoseFilter>());rclcpp::shutdown();return 0;
}
