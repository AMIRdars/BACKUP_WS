#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "amir_interfaces/msg/amir_sensor.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

class CameraToBaseNode : public rclcpp::Node
{
public:
  CameraToBaseNode()
  : Node("camera_to_base_node")
  {
    rclcpp::QoS qos_profile(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data));
    qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);

    joint_angle_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>(
      "/encoder_pub", qos_profile,
      std::bind(&CameraToBaseNode::angle_callback, this, std::placeholders::_1));
    object_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
      "/bottle_position", 10, std::bind(&CameraToBaseNode::object_callback, this, std::placeholders::_1));

    object_sub_score = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/score_position", 10, std::bind(&CameraToBaseNode::object_callback2, this, std::placeholders::_1));
  

    object_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/amir1/object_position", 10);
    object_pub_2 = this->create_publisher<geometry_msgs::msg::PoseStamped>("/amir1/score_position2", 10);

    RCLCPP_INFO(this->get_logger(), "CameraToBaseNode started");
  }

private:
  rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr joint_angle_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr object_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr object_pub_2;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr object_sub_score;

  double theta1_{0.0}, theta2_{0.0}, theta3_{0.0};

  const double L0 = 0.036;
  const double L1 = 0.092;
  const double L2 = 0.31;
  const double L3 = 0.31;
  const double D = 0.321;

  // 最大の関節角度
  float deg_max_1 = 340; // 0 to ~
  float deg_max_2 = -135;
  float deg_max_3 = 160;

  float theta1_rad; // 負から0
  float theta2_rad; // 負から0
  float theta3_rad; // 0から正

  float off_theta1_deg;
  float off_theta2_deg;
  float off_theta3_deg;

  const Eigen::Vector3d t_cam = Eigen::Vector3d(0.245, 0.06, 0.065); //第３関節から見たカメラの位置

  void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
  {
    theta1_ = msg->angle[0] / 1000.0;
    theta2_ = msg->angle[1] / 1000.0;
    theta3_ = msg->angle[2] / 1000.0;

    float theta1_deg = ((theta1_ * 180) / (M_PI));
    float theta2_deg = ((theta2_ * 180) / (M_PI));
    float theta3_deg = ((theta3_ * 180) / (M_PI));

    
    // float off_theta1_deg_result = std::clamp(theta1_deg, deg_max_1 / 2, -deg_max_1 / 2); // 負から0
    // float off_theta2_deg_result = std::clamp(theta2_deg, 0.0f, -deg_max_2); // 負から0
    // float off_theta3_deg_result = std::clamp(theta3_deg, -deg_max_3, 0.0f); // 0から正

    // RCLCPP_INFO(this->get_logger(), "Angle: 1=%f, 2=%f, 3=%f", off_theta1_deg_result, off_theta2_deg_result, off_theta3_deg_result);

    off_theta1_deg = (deg_max_1 / 2) + theta1_deg;
    off_theta2_deg = deg_max_2 - theta2_deg;
    off_theta3_deg = deg_max_3 - theta3_deg;

    // RCLCPP_INFO(this->get_logger(), "Angle: 1=%f, 2=%f, 3=%f", off_theta1_deg, off_theta2_deg, off_theta3_deg);

    // radに変更
    theta1_rad = (off_theta1_deg * M_PI) / 180;
    theta2_rad = (off_theta2_deg * M_PI) / 180;
    theta3_rad = (off_theta3_deg * M_PI) / 180;
    
  }

  void object_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    Eigen::Vector3d p_cam(msg->point.x, msg->point.y, msg->point.z);

    Eigen::Matrix3d R_cv_to_ros;
    R_cv_to_ros << 
      0,  0, 1,
     -1,  0, 0,
      0, -1, 0;

    Eigen::Affine3d T_base_to_cam =
      Eigen::Translation3d(L0, 0, 0) *
      Eigen::AngleAxisd(theta1_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::Translation3d(0, 0, L1) *
      Eigen::AngleAxisd(theta2_rad, Eigen::Vector3d::UnitY()) *
      Eigen::Translation3d(L2, 0, 0) *
      Eigen::AngleAxisd(theta3_rad, Eigen::Vector3d::UnitY()) *
      Eigen::Translation3d(t_cam) *
      Eigen::Affine3d(R_cv_to_ros);

    Eigen::Vector3d p_base = T_base_to_cam * p_cam;

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = this->get_clock()->now();
    out.header.frame_id = "amir_base";
    out.pose.position.x = p_base.x();
    out.pose.position.y = p_base.y();
    out.pose.position.z = p_base.z() + 0.15;

    object_pub_->publish(out);
    // RCLCPP_INFO(this->get_logger(), "Angle: 1=%f, 2=%f, 3=%f", off_theta1_deg, off_theta2_deg, off_theta3_deg);
    RCLCPP_INFO(this->get_logger(), "x=%f, y=%f, z=%f", out.pose.position.x, out.pose.position.y, out.pose.position.z);
  }

  void object_callback2(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    Eigen::Vector3d p_cam(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    Eigen::Matrix3d R_cv_to_ros;
    R_cv_to_ros << 
      0,  0, 1,
     -1,  0, 0,
      0, -1, 0;

    Eigen::Affine3d T_base_to_cam =
      Eigen::Translation3d(L0, 0, 0) *
      Eigen::AngleAxisd(theta1_rad, Eigen::Vector3d::UnitZ()) *
      Eigen::Translation3d(0, 0, L1) *
      Eigen::AngleAxisd(theta2_rad, Eigen::Vector3d::UnitY()) *
      Eigen::Translation3d(L2, 0, 0) *
      Eigen::AngleAxisd(theta3_rad, Eigen::Vector3d::UnitY()) *
      Eigen::Translation3d(t_cam) *
      Eigen::Affine3d(R_cv_to_ros);

    Eigen::Vector3d p_base = T_base_to_cam * p_cam;

    geometry_msgs::msg::PoseStamped out;
    out.header.stamp = this->get_clock()->now();
    out.header.frame_id = "amir_base";
    out.pose.position.x = p_base.x();
    out.pose.position.y = p_base.y();
    out.pose.position.z = p_base.z() + 0.15;

    object_pub_2->publish(out);
    // RCLCPP_INFO(this->get_logger(), "Angle: 1=%f, 2=%f, 3=%f", off_theta1_deg, off_theta2_deg, off_theta3_deg);
    RCLCPP_INFO(this->get_logger(), "x=%f, y=%f, z=%f", out.pose.position.x, out.pose.position.y, out.pose.position.z);
  }



};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CameraToBaseNode>());
  rclcpp::shutdown();
  return 0;
}