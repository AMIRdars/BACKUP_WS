#include <algorithm>
#include <array>
#include <chrono>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/create_timer.hpp>
#include <stdexcept>
#include <limits>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <ros_gz_interfaces/msg/contacts.hpp>

// Keep per-segment states separate: an empty segment must not clear contact
// on another segment of the same finger. Force feedback remains untouched.
class FingerContactAggregator : public rclcpp::Node {
 struct Segment {bool touching=false;double seen=0;double depth=0;bool received=false;bool has_depth=false;};
 struct Finger {
   std::string robot,side;std::vector<Segment> segments;
   rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher;
 };
 public:
  FingerContactAggregator():Node("finger_contact_aggregator") {
    const auto robots=declare_parameter<std::vector<std::string>>("robot_names",{"amir1","amir2"});
    const int count=declare_parameter<int>("collision_count",9);
    const double rate=declare_parameter<double>("publish_rate",30.0);
    timeout_=declare_parameter<double>("contact_timeout",0.25);
    payload_=declare_parameter<std::string>("payload_token","cooperative_payload");
    if(count<=0||rate<=0||timeout_<=0)throw std::invalid_argument("positive count/rate/timeout required");
    for(const auto &robot:robots)for(const auto &side:{std::string("left"),std::string("right")}) {
      Finger finger;finger.robot=robot;finger.side=side;finger.segments.resize(count);
      finger.publisher=create_publisher<std_msgs::msg::Bool>("/"+robot+"/finger_"+side+"_contact_detected",10);
      fingers_.push_back(std::move(finger));
    }
    for(size_t f=0;f<fingers_.size();++f)for(int s=0;s<count;++s) {
      const auto &finger=fingers_[f];
      subscriptions_.push_back(create_subscription<ros_gz_interfaces::msg::Contacts>(
        "/"+finger.robot+"/finger_"+finger.side+"_contact_segments/segment_"+std::to_string(s),rclcpp::QoS(1),
        [this,f,s](const ros_gz_interfaces::msg::Contacts::SharedPtr message) {
          auto &finger=fingers_[f];auto &segment=finger.segments[s];
          segment.touching=false;segment.depth=0;segment.has_depth=false;segment.seen=now().seconds();segment.received=true;
          for(const auto &contact:message->contacts) {
            const std::string token="finger_"+finger.side+"_1";
            const auto is_finger=[&](const std::string &name){return name.find(finger.robot)!=std::string::npos&&name.find(token)!=std::string::npos;};
            const auto is_payload=[&](const std::string &name){return name.find(payload_)!=std::string::npos;};
            if((is_finger(contact.collision1.name)&&is_payload(contact.collision2.name))||
               (is_finger(contact.collision2.name)&&is_payload(contact.collision1.name))) {
              segment.touching=true;
              for(double depth:contact.depths){segment.has_depth=true;segment.depth=std::max(segment.depth,depth);}
            }
          }
        }));
    }
    diagnostics_=create_publisher<std_msgs::msg::Float64MultiArray>("/cooperative_transport/contact_metrics",10);
    timer_=rclcpp::create_timer(this,get_clock(),std::chrono::duration<double>(1.0/rate),[this](){publish();});
  }
 private:
  void publish() {
    const double time=now().seconds();std_msgs::msg::Float64MultiArray metrics;
    for(auto &finger:fingers_) {
      int count=0;double depth=0;bool has_depth=false;
      for(const auto &segment:finger.segments)if(segment.received&&time>=segment.seen&&time-segment.seen<=timeout_&&segment.touching){++count;if(segment.has_depth){has_depth=true;depth=std::max(depth,segment.depth);}}
      std_msgs::msg::Bool message;message.data=count>0;finger.publisher->publish(message);
      metrics.data.push_back(count);metrics.data.push_back(count>0&&!has_depth?std::numeric_limits<double>::quiet_NaN():depth);
    }
    diagnostics_->publish(metrics);
  }
  std::string payload_;double timeout_;
  std::vector<Finger> fingers_;
  std::vector<rclcpp::Subscription<ros_gz_interfaces::msg::Contacts>::SharedPtr> subscriptions_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr diagnostics_;
  rclcpp::TimerBase::SharedPtr timer_;
};
int main(int argc,char **argv) {rclcpp::init(argc,argv);rclcpp::spin(std::make_shared<FingerContactAggregator>());rclcpp::shutdown();return 0;}
