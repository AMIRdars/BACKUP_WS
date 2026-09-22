#include <iostream>
#include <stdexcept>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behavior_tree_msgs/msg/bt_status.hpp"
#include "behavior_tree_msgs/srv/get_bt.hpp"
#include "behavior_tree_msgs/srv/set_black_board.hpp"
#include "behavior_tree_msgs/action/execute_tree.hpp"

#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/create_timer_ros.h"
// #include "behaviortree_cpp_v3/loggers/bt_cout_logger.h"
// #include "behaviortree_cpp_v3/loggers/bt_file_logger.h"
// #include "behaviortree_cpp_v3/loggers/bt_minitrace_logger.h"
// #include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
// The Behavior Tree to execute. This tree simple prints "Hello," and
// "World!" on separate lines and then terminates.



using namespace std::chrono_literals;


class BTExecutorClass
{
  public:
    BTExecutorClass(rclcpp::Node::SharedPtr node):node_(node)
    {
      using std::placeholders::_1;
      using std::placeholders::_2;  

      if (!node->has_parameter("bt_loop_duration")) node->declare_parameter("bt_loop_duration", 10);
      if (!node->has_parameter("default_server_timeout")) node->declare_parameter("default_server_timeout", 20);
      if (!node->has_parameter("enable_groot_monitoring")) node->declare_parameter<bool>("enable_groot_monitoring", false);
      if (!node->has_parameter("groot_zmq_publisher_port")) node->declare_parameter<int>("groot_zmq_publisher_port", 1666);
      if (!node->has_parameter("groot_zmq_server_port")) node->declare_parameter<int>("groot_zmq_server_port", 1667);
      if (!node->has_parameter("plugin_lib_names")) node->declare_parameter<std::vector<std::string>>("plugin_lib_names", std::vector<std::string>(0,""));
      node->get_parameter("bt_loop_duration", bt_loop_duration_);
      node->get_parameter("default_server_timeout", default_server_timeout_);
      node->get_parameter("enable_groot_monitoring", enable_groot_monitoring_);
      node->get_parameter("groot_zmq_publisher_port", groot_zmq_publisher_port_);
      node->get_parameter("groot_zmq_server_port", groot_zmq_server_port_);
      node->get_parameter("plugin_lib_names", plugin_lib_names_);

      publisher_ = node->create_publisher<behavior_tree_msgs::msg::BTStatus>("bt_status", 10);

      /*BTのツリーを受け取るサービス*/
      service_server_ = node->create_service<behavior_tree_msgs::srv::GetBT>("bt_update", std::bind(&BTExecutorClass::receive_bt, this, _1, _2));
      bb_server_ = node->create_service<behavior_tree_msgs::srv::SetBlackBoard>("set_blackboard", std::bind(&BTExecutorClass::bb_cb, this, _1, _2));
      

      /*プラグインの読込確認用*/
      for (auto x : plugin_lib_names_)
      {
        std::cout << x << std::endl;
      }

      /*tf関連*/
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_->get_node_base_interface(), node_->get_node_timers_interface());
      tf_buffer_->setCreateTimerInterface(timer_interface);
      tf_buffer_->setUsingDedicatedThread(true);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);


      /*BehaviorTreeの初期化*/
      static const char bt_xml[] =
        R"(
      <root main_tree_to_execute="MainTree">
        <BehaviorTree ID="MainTree"/>
      </root>
      )";

      
      bt_ = std::make_unique<ros2_behavior_tree::BehaviorTree>(bt_xml, plugin_lib_names_);
      
      // BT::Blackboard::Ptr blackboard = bt_->blackboard();
      blackboard_ = bt_->blackboard();

      /*nav2用プラグインはこれらを設定する必要がある*/
      blackboard_->set<rclcpp::Node::SharedPtr>("node", node);  // NOLINT
      blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);  // NOLINT
      blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(default_server_timeout_));  // NOLINT
      blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(bt_loop_duration_));  // NOLINT
  


      
      /*pick and placeテスト用*/
      geometry_msgs::msg::PoseStamped test_obj_pose, test_location_pose;
      test_obj_pose.pose.position.x = 3.0;
      test_obj_pose.pose.position.z = 0.15;
      test_location_pose.pose.position.y = 5.0;
      test_obj_pose.header.frame_id = "map";
      test_location_pose.header.frame_id = "map";

      blackboard_->set<geometry_msgs::msg::PoseStamped>("test_obj_pose", test_obj_pose);  // NOLINT
      blackboard_->set<geometry_msgs::msg::PoseStamped>("test_location_pose", test_location_pose);  // NOLINT


      /*ツリーの生成*/
      // bt_->update_xml(bt_xml);
      bt_root_status_ = BT::NodeStatus::RUNNING;

      /*ツリーの実行*/
      timer_ = node->create_wall_timer(
        500ms, std::bind(&BTExecutorClass::timer_callback, this));
      rclcpp::spin(node);
     
    }

    void timer_callback()
    {
      if(!bt_->set_tree) return;// treeがセットしてなければ終了

      /*結果がまだならtickする*/
      if(bt_root_status_ == BT::NodeStatus::IDLE || bt_root_status_ == BT::NodeStatus::RUNNING)
      {
        std::vector<BT::TreeNode::Ptr> bt_nodes;
        bt_->executeTick(bt_root_status_, bt_nodes);// tick
      }
      else if (bt_root_status_ == BT::NodeStatus::FAILURE)
      {
        RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "BT root is FAILURE");
        
      }

      bt_->getStatus(bt_status_); //  ノードの状態を取得

      publisher_->publish(bt_status_); 
    }

    /*ツリーを受信したときのコールバック関数*/
    void receive_bt(const std::shared_ptr<behavior_tree_msgs::srv::GetBT::Request> request,
          std::shared_ptr<behavior_tree_msgs::srv::GetBT::Response>      response)
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BT received");
      // std::cout << request->bt << std::endl;

      bt_->update_xml(request->bt);  // BTのツリーを更新
      bt_->addStatusPub();  // BTの状態遷移をpublishする用
      bt_root_status_ = BT::NodeStatus::RUNNING;
      response->behavior_tree;
    }

    /*BlackBoard書き込み用サービスコールバック*/
    void bb_cb(const std::shared_ptr<behavior_tree_msgs::srv::SetBlackBoard::Request> request,
          std::shared_ptr<behavior_tree_msgs::srv::SetBlackBoard::Response>      response)
    { 
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set_black_board");
      for(auto pose: request->poses)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set %s",pose.key.c_str());
        blackboard_->set<geometry_msgs::msg::PoseStamped>(pose.key, pose.pose);  // NOLINT
      }

      for(auto path: request->paths)
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set %s",path.key.c_str());
        blackboard_->set<nav_msgs::msg::Path>(path.key, path.path);  // NOLINT
      }
      

    }

    

  private:
    std::vector<std::string> plugin_lib_names_;
    bool enable_groot_monitoring_;
    int bt_loop_duration_;
    int default_server_timeout_;
    int groot_zmq_publisher_port_;
    int groot_zmq_server_port_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<behavior_tree_msgs::msg::BTStatus>::SharedPtr publisher_;
    rclcpp::Service<behavior_tree_msgs::srv::GetBT>::SharedPtr service_server_;
    rclcpp::Service<behavior_tree_msgs::srv::SetBlackBoard>::SharedPtr bb_server_;
    
    std::unique_ptr<ros2_behavior_tree::BehaviorTree> bt_;
    BT::NodeStatus bt_root_status_;
    behavior_tree_msgs::msg::BTStatus bt_status_;
    // size_t count_;
    // std::unique_ptr<std::thread> thread_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    BT::Blackboard::Ptr blackboard_;
    // rclcpp::Clock ros_clock_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("bt_executor");
  BTExecutorClass test(node);
  
  rclcpp::shutdown();
  return 0;
}
