#include "coop_transport_bt/follower_compliant_carry.hpp"
#include <chrono>

namespace coop_transport_bt
{
FollowerCompliantCarry::FollowerCompliantCarry(
  const std::string &name,const BT::NodeConfiguration &config)
: BT::StatefulActionNode(name,config)
{
  if (!config.blackboard || !config.blackboard->get("node",node_)) {
    throw BT::RuntimeError("FollowerCompliantCarry needs blackboard node");
  }
}
BT::PortsList FollowerCompliantCarry::providedPorts()
{
  return {BT::InputPort<std::string>("robot_namespace",std::string("amir2"),"local robot namespace"),
          BT::InputPort<double>("duration_s",0.,"zero: run until halted"),
          BT::InputPort<double>("status_timeout_s",.5,"status age limit")};
}
BT::NodeStatus FollowerCompliantCarry::onStart()
{
  std::string ns;
  if (!getInput("robot_namespace",ns) || ns.empty()) {return BT::NodeStatus::FAILURE;}
  if (ns.front()!='/') {ns="/"+ns;}
  client_=node_->create_client<std_srvs::srv::SetBool>(ns+"/follower/enable");
  status_.clear();
  status_sub_=node_->create_subscription<std_msgs::msg::String>(ns+"/follower/status",10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      status_=msg->data; status_at_=node_->now();
    });
  started_=node_->now();
  accepted_=false;
  stopping_=false;
  request_sent_=false;
  // With use_sim_time, the first /clock may arrive after onStart. Starting
  // the discovery deadline at zero would turn that first sample into a timeout.
  clock_ready_=!node_->get_parameter("use_sim_time").as_bool() || started_.nanoseconds()>0;
  return BT::NodeStatus::RUNNING;
}
void FollowerCompliantCarry::disable()
{
  if (client_ && client_->service_is_ready()) {
    auto request=std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data=false;
    client_->async_send_request(request);
  }
}
BT::NodeStatus FollowerCompliantCarry::onRunning()
{
  double timeout=.5,duration=0.;
  getInput("status_timeout_s",timeout);
  getInput("duration_s",duration);
  if (!clock_ready_) {
    if (node_->now().nanoseconds()==0) {return BT::NodeStatus::RUNNING;}
    started_=node_->now();
    clock_ready_=true;
  }
  const auto elapsed=(node_->now()-started_).seconds();
  if (stopping_) {
    if (stop_response_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      if (!stop_response_.get()->success) {return BT::NodeStatus::FAILURE;}
      if (status_=="DISABLED") {return BT::NodeStatus::SUCCESS;}
    }
    if (status_=="FAULT" || (node_->now()-stop_at_).seconds()>2.) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::RUNNING;
  }
  if (!accepted_) {
    if (!request_sent_) {
      if (client_->service_is_ready()) {
        auto request=std::make_shared<std_srvs::srv::SetBool::Request>();
        request->data=true;
        response_=client_->async_send_request(request).share();
        request_sent_=true;
      } else if (elapsed>2.) {return BT::NodeStatus::FAILURE;}
      return BT::NodeStatus::RUNNING;
    }
    if (response_.wait_for(std::chrono::seconds(0))==std::future_status::ready) {
      const auto response=response_.get();
      if (!response->success) {
        RCLCPP_ERROR(node_->get_logger(),"Follower enable refused: %s",response->message.c_str());
        disable(); return BT::NodeStatus::FAILURE;
      }
      accepted_=true;
      enabled_at_=node_->now();
    } else if (elapsed>2.) {disable(); return BT::NodeStatus::FAILURE;}
    else {return BT::NodeStatus::RUNNING;}
  }
  const auto active_elapsed=(node_->now()-enabled_at_).seconds();
  if (status_=="FAULT" || elapsed<0 ||
      (active_elapsed>timeout && status_!="FOLLOWING" && status_!="RECENTERING") ||
      (active_elapsed>timeout && (status_.empty() || (node_->now()-status_at_).seconds()>timeout))) {
    disable(); return BT::NodeStatus::FAILURE;
  }
  if (duration>0 && active_elapsed>=duration) {
    if (!client_->service_is_ready()) {return BT::NodeStatus::FAILURE;}
    auto request=std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data=false;
    stop_response_=client_->async_send_request(request).share();
    stop_at_=node_->now();
    stopping_=true;
    return BT::NodeStatus::RUNNING;
  }
  return BT::NodeStatus::RUNNING;
}
void FollowerCompliantCarry::onHalted() {disable();}
}
