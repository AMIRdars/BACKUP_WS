#include <iostream>
#include <stdexcept>
#include <unordered_map> 
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp/time.hpp"
#include "ros2_behavior_tree/behavior_tree.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "behavior_tree_msgs/msg/bt_status.hpp"
#include "behavior_tree_msgs/srv/get_bt.hpp"
#include "behavior_tree_msgs/srv/set_black_board.hpp"
#include "behavior_tree_msgs/srv/create_bt.hpp"
#include "behavior_tree_msgs/action/execute_tree.hpp"
#include "nav2_util/simple_action_server.hpp"

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
    using Action = behavior_tree_msgs::action::ExecuteTree;
    using ActionServer = nav2_util::SimpleActionServer<Action>;

    BTExecutorClass(rclcpp::Node::SharedPtr node, std::string bt_name):node_(node),bt_name_(bt_name)
    {
      using namespace std::placeholders;

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

      publisher_ = node->create_publisher<behavior_tree_msgs::msg::BTStatus>("bt_status_" + bt_name_, 10);


      // this->action_server_ = rclcpp_action::create_server<ExecuteTree>(
      //   node_,
      //   "execute_bt",
      //   std::bind(&BTExecutorClass::handle_goal, this, _1, _2),
      //   std::bind(&BTExecutorClass::handle_cancel, this, _1),
      //   std::bind(&BTExecutorClass::handle_accepted, this, _1));

      action_server_ = std::make_unique<ActionServer>(
        node_, "execute_" + bt_name_,
        std::bind(&BTExecutorClass::execute_cb, this));

      /*BTのツリーを受け取るサービス*/
      service_server_ = node->create_service<behavior_tree_msgs::srv::GetBT>("update_" + bt_name_, std::bind(&BTExecutorClass::receive_bt, this, _1, _2));
      bb_server_ = node->create_service<behavior_tree_msgs::srv::SetBlackBoard>("set_blackboard_" + bt_name_, std::bind(&BTExecutorClass::bb_cb, this, _1, _2));
      

      /*プラグインの読込確認用*/
      if (plugin_lib_names_.size() == 0) RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "plugin_lib is empty");
      // for (auto x : plugin_lib_names_)
      // {
      //   std::cout << x << std::endl;
      // }

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

      
      bt_ = std::make_unique<ros2_behavior_tree::BehaviorTree>(bt_xml, plugin_lib_names_, bt_name_);
      // bt_->register_tree("/home/kawase/galactic_ws/src/bt_generator/bt_xml/subtree.xml");
      
      // BT::Blackboard::Ptr blackboard = bt_->blackboard();
      blackboard_ = bt_->blackboard();

      /*nav2用プラグインはこれらを設定する必要がある*/
      blackboard_->set<rclcpp::Node::SharedPtr>("node", node);  // nav_plugin用
      blackboard_->set<rclcpp::Node::SharedPtr>("ros2_node", node);  // NOLINT  
      blackboard_->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer_);  // NOLINT
      blackboard_->set<std::chrono::milliseconds>("server_timeout", std::chrono::milliseconds(default_server_timeout_));  // NOLINT
      blackboard_->set<std::chrono::milliseconds>("bt_loop_duration", std::chrono::milliseconds(bt_loop_duration_));  // NOLINT



      
      /*pick and placeテスト用*/
      // geometry_msgs::msg::PoseStamped test_obj_pose, test_location_pose;
      // test_obj_pose.pose.position.x = 3.0;
      // test_obj_pose.pose.position.z = 0.15;
      // test_location_pose.pose.position.y = 5.0;
      // test_obj_pose.header.frame_id = "map";
      // test_location_pose.header.frame_id = "map";

      // blackboard_->set<geometry_msgs::msg::PoseStamped>("test_obj_pose", test_obj_pose);  // NOLINT
      // blackboard_->set<geometry_msgs::msg::PoseStamped>("test_location_pose", test_location_pose);  // NOLINT


      /*ツリーの生成*/
      // bt_->update_xml(bt_xml);
      bt_root_status_ = BT::NodeStatus::RUNNING;

      // /*ツリーの実行*/
      // timer_ = node->create_wall_timer(
      //   500ms, std::bind(&BTExecutorClass::timer_callback, this));
      action_server_->activate();
      // rclcpp::spin(node);
     
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


    void execute_cb()
    {
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "BT received");
      

      rclcpp::Rate loop_rate(2);  //2Hz
      auto feedback = std::make_shared<Action::Feedback>();
      auto result = std::make_shared<Action::Result>();
      // std::cout << request->bt << std::endl;

      bt_->update_xml(action_server_->get_current_goal()->bt);  // BTのツリーを更新
      bt_->addStatusPub();  // BTの状態遷移をpublishする用

      bt_root_status_ = BT::NodeStatus::RUNNING;

      while(bt_root_status_ == BT::NodeStatus::IDLE || bt_root_status_ == BT::NodeStatus::RUNNING && rclcpp::ok())
      {
        //  アクティベートされてない場合は抜ける
        if (action_server_ == nullptr || !action_server_->is_server_active()) {
          RCLCPP_DEBUG(rclcpp::get_logger("rclcpp"), "Action server unavailable or inactive. Stopping.");
          return;
        }

        //  キャンセル時の処理
        if (action_server_->is_cancel_requested()) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Goal was canceled. Stopping the robot.");
          action_server_->terminate_all();
          bt_->haltTree(); //  BT停止
          return;
        }

        //  新しいツリーが送られたときの処理
        if (action_server_->is_preempt_requested()) {
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Passing new tree.");
          auto goal = action_server_->accept_pending_goal();
          std::string current_controller;
          bt_->update_xml(goal->bt);  // BTのツリーを更新
          bt_->addStatusPub();  // BTの状態遷移をpublishする用
        }

        std::vector<BT::TreeNode::Ptr> bt_nodes;
        bt_->executeTick(bt_root_status_, bt_nodes);// tick

        bt_->getStatus(bt_status_); //  ノードの状態を取得
        feedback->bt_status = bt_status_;
        // goal_handle->publish_feedback(feedback);
        action_server_->publish_feedback(feedback);
        loop_rate.sleep();
      }

      
      // Check if goal is done
      if (rclcpp::ok()) {
        if(bt_root_status_ == BT::NodeStatus::SUCCESS)
        {
          // result->bt_status.root_status.status = behavior_tree_msgs::msg::NodeStatus::SUCCESS;
          bt_->getStatus(bt_status_); //  ノードの状態を取得
          result->bt_status = bt_status_;
          result->error_string = "success";
        }
        else 
        {
          result->error_string = "failure";
        }
        

        // goal_handle->succeed(result);
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finished %s",bt_name_.c_str());
        action_server_->succeeded_current(result);
      }
     
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
      for(auto pose: request->bb_message.poses)
      {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set %s",pose.key.c_str());
        blackboard_->set<geometry_msgs::msg::PoseStamped>(pose.key, pose.value);  // NOLINT
      }

      for(auto path: request->bb_message.paths)
      {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set %s",path.key.c_str());
        blackboard_->set<nav_msgs::msg::Path>(path.key, path.value);  // NOLINT
      }

      for(auto float_array: request->bb_message.float_array_set)
      {
        // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "set %s",float_array.key.c_str());
        blackboard_->set<std_msgs::msg::Float64MultiArray>(float_array.key, float_array.value);  // NOLINT
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
    // rclcpp_action::Server<ExecuteTree>::SharedPtr action_server_;
    std::unique_ptr<ActionServer> action_server_;
    
    std::unique_ptr<ros2_behavior_tree::BehaviorTree> bt_;
    BT::NodeStatus bt_root_status_;
    behavior_tree_msgs::msg::BTStatus bt_status_;
    // size_t count_;
    // std::unique_ptr<std::thread> thread_;
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    BT::Blackboard::Ptr blackboard_;
    bool tree_updated_;
    std::string bt_name_;
    // rclcpp::Clock ros_clock_;

};


class TestClass
{
  using Action = behavior_tree_msgs::action::ExecuteTree;
  using ActionServer = nav2_util::SimpleActionServer<Action>;
  public:
    TestClass(rclcpp::Node::SharedPtr node):node_(node)
    {

      /*tf関連*/
      tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node_->get_clock());
      auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        node_->get_node_base_interface(), node_->get_node_timers_interface());
      tf_buffer_->setCreateTimerInterface(timer_interface);
      tf_buffer_->setUsingDedicatedThread(true);
      tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, node_, false);
    }
  private:
    rclcpp::Node::SharedPtr node_;
    std::unique_ptr<ActionServer> action_server_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Publisher<behavior_tree_msgs::msg::BTStatus>::SharedPtr publisher_;
    BT::Blackboard::Ptr blackboard_;
};



class CreateBTClass
{
  // typedef std::unordered_map<std::string, BTExecutorClass>::value_type map_value_type;
  public:
    CreateBTClass()
    {
      using namespace std::placeholders;
      
      std::string main_bt_name = "bt_executor";
      auto node = rclcpp::Node::make_shared(main_bt_name);  
      rclcpp::Service<behavior_tree_msgs::srv::CreateBT>::SharedPtr service_server_ 
        = node->create_service<behavior_tree_msgs::srv::CreateBT>("create_bt", std::bind(&CreateBTClass::create_bt_cb, this, _1, _2));

      
      std::cout << node->get_name() << std::endl;
      BTExecutorClass main_executor(node, "bt");

      // bt_executors_["bt1"] = test1;
      exec_.add_node(node); //  エグゼキュータに追加  
      /////////////////////////////////

      
      exec_.spin();

    }
    ~CreateBTClass() {}


  private:
    void create_bt_cb(const std::shared_ptr<behavior_tree_msgs::srv::CreateBT::Request> request,
          std::shared_ptr<behavior_tree_msgs::srv::CreateBT::Response> response)
    {
      
      // BTExecutorClass executor(node, request->bt_name);
      // bt_executors_.push_back(std::move(executor));  //  クラスにstd::unique_ptrがあるとstd::moveが必要らしい

      // bt_executors_[request->bt_name] = std::move(executor);

      //  まだbtのオブジェクトが生成されていない場合
      if(bt_executors_.find(request->bt_name) == bt_executors_.end())
      {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "create %s",request->bt_name.c_str());
        auto node = rclcpp::Node::make_shared(request->bt_name);
        bt_executors_[request->bt_name] = std::make_unique<BTExecutorClass>(node, request->bt_name);  //  クラス生成して保管
        exec_.add_node(node); //  エグゼキュータに追加  
        response->error_string = "success";
      }
      else  response->error_string = "exist";
    }

    rclcpp::executors::MultiThreadedExecutor exec_;
    // std::vector<BTExecutorClass> bt_executors_;
    std::unordered_map<std::string, std::unique_ptr<BTExecutorClass>> bt_executors_;
    
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  CreateBTClass create_bt_class;

  



  

  // rclcpp::spin(node);
  
  rclcpp::shutdown();
  return 0;
}
