#include <rclcpp/rclcpp.hpp>
#include <amir_interfaces/msg/amir_cmd.hpp>
#include "std_msgs/msg/empty.hpp"
#include <string>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>
#include <chrono>
#include <thread>
#include "amir_interfaces/msg/amir_sensor.hpp"

class JointPublisher : public rclcpp::Node
{
public:
  JointPublisher()
  : Node("vis_joint_publisher")
  { 
    rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
    qos_profile.best_effort();
    joint_pub_ = this->create_publisher<amir_interfaces::msg::AmirCmd>("/motor_sub", qos_profile);
    move_arm_sub_ = this->create_subscription<amir_interfaces::msg::AmirSensor>("/encoder_pub", qos_profile, std::bind(&JointPublisher::angle_callback, this, std::placeholders::_1
    ));
    // publish_joint_states(); // 一度だけ呼び出す
    timer_ = this->create_wall_timer(
      std::chrono::seconds(1),
      std::bind(&JointPublisher::publish_joint_states, this)
    );

  }

private:
  float P_x{0.0f}, P_y{0.0f}, P_z{0.0f};
  rclcpp::Publisher<amir_interfaces::msg::AmirCmd>::SharedPtr joint_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<amir_interfaces::msg::AmirSensor>::SharedPtr move_arm_sub_;
  // double current_angle1;
  // double current_angle2;
  // double current_angle3;
  // double current_angle4;
  // double current_angle5;
  // double current_angle6;
  double current_angle1{0.0}, current_angle2{0.0}, current_angle3{0.0},
         current_angle4{0.0}, current_angle5{0.0}, current_angle6{0.0};

  void angle_callback(const amir_interfaces::msg::AmirSensor::SharedPtr msg)
  {
      // RCLCPP_INFO(this->get_logger(), "current_angl6");
      //取得した各関節角をcurrent_angle1~5に入れる
      current_angle1 = msg->angle[0];
      current_angle2 = msg->angle[1];
      current_angle3 = msg->angle[2];
      current_angle4 = msg->angle[3];
      current_angle5 = msg->angle[4];
      current_angle6 = msg->angle[5];
      // RCLCPP_INFO(this->get_logger(), "current_angl6:%f", current_angl6);
  }
  void publish_joint_states()
  {     

    std::cout << "target x y z を入力: ";
    if (!(std::cin >> P_x >> P_y >> P_z)) {
      std::cin.clear();           // エラークリア
      std::cin.ignore(100, '\n');
      RCLCPP_WARN(this->get_logger(), "入力が不正です。再度入力してください。");
      return;
    }
    // 逆運動学計算
        // RCLCPP_INFO(this->get_logger(), "Start inverse kinematics with x: %f, y: %f, z: %f", P_x, P_y, P_z);

        //amir740のリンクの長さと各関節角の入力と各変数の定義
        float L0 = 0.036;
        float L1 = 0.092;
        float L2 = 0.31;
        float L3 = 0.31;
        float L4 = 0.038;
        float L5 = 0.088;
        float D = 0.321;

        // 最大の関節角度
        float deg_max_1 = -340; // 0 to ~
        float deg_max_2 = -135;
        float deg_max_3 = 160;
        float deg_max_4 = -195;
        float deg_max_5 = 316;

        float theta234;
        float theta234_;

        float a;
        float b;
        float c;
        float d;

        float x;
        float z;
        
        //単位行列とする
        // float r_11 = 1.0;
        // float r_12 = 0.0;
        // float r_13 = 0.0;
        // float r_21 = 0.0;
        // float r_22 = 1.0;
        // float r_23 = 0.0;
        // float r_31 = 0.0;
        // float r_32 = 0.0;
        // float r_33 = 1.0;
        ///x
        // float r_11 = 1.0;
        // float r_12 = 0.0;
        // float r_13 = 0.0;
        // float r_21 = 0.0;
        // float r_22 = 0.0;
        // float r_23 = -1.0;
        // float r_31 = 0.0;
        // float r_32 = 1.0;
        // float r_33 = 0.0;
        //y
        // float r_11 = 0.0;
        // float r_12 = 0.0;
        // float r_13 = 1.0;
        // float r_21 = 0.0;
        // float r_22 = 1.0;
        // float r_23 = 0.0;
        // float r_31 = -1.0;
        // float r_32 = 0.0;
        // float r_33 = 0.0;
        float r_11 = 0.0;
        float r_12 = -1.0;
        float r_13 = 0.0;
        float r_21 = 1.0;
        float r_22 = 0.0;
        float r_23 = 0.0;
        float r_31 = 0.0;
        float r_32 = 0.0;
        float r_33 = 1.0;


        // rad出力
        float theta1;
        float theta2;
        float theta3;
        float theta4;
        float theta5;
        
        // deg出力
        float theta1_deg;
        float theta2_deg;
        float theta3_deg;
        float theta4_deg;
        float theta5_deg;

        // mrad出力
        float theta1_mrad;
        float theta2_mrad;
        float theta3_mrad;
        float theta4_mrad;
        float theta5_mrad;

        // 目標位置にて入力した値と順運動学より出力した値の誤差計算
        float error_x;
        float error_y;
        float error_z;
        float error_r_11;
        float error_r_22;
        float error_r_33;

        //逆運動学の計算に必要な変数の計算
        theta1 = (atan2(P_y, P_x));
        theta234 = atan((cos(theta1) * r_13+sin(theta1) * r_23) / r_33);
        theta234_ = - theta234 - (M_PI) / 2;
        x = (P_x / cos(theta1)) - L0;
        z = P_z - (D + L1);
        a = z - (L4 + L5) * sin(theta234_);
        b = x - (L4 + L5) * cos(theta234_);
        c = ((b * b) + (a * a) + (L2 * L2) - (L3 * L3)) / (2 * L2);
        d = ((b * b) + (a * a) + (L3 * L3) - (L2 * L2)) / (2 * L2);

        //逆運動学でamirの各関節角度を計算する
        // 各thetaの範囲内であれば各関節角度を計算する
        theta2 = (acos(c / (sqrt(a * a + b * b))) + acos(b / (sqrt(a * a + b * b))));
        theta3 = (-M_PI + 2 * (asin(c / (sqrt((a * a) + (b * b))))));
        theta4 = ((atan2((cos(theta1) * r_13 + sin(theta1) * r_23),r_33) - theta2 - theta3));
        theta5 = (atan2((cos(theta1) * r_21 - sin(theta1) * r_11),(cos(theta1) * r_22 - sin(theta1) * r_12)));

        // 順運動学にて出力する位置座標と姿勢行列の値
        float pos_x;
        float pos_y;
        float pos_z;
        float r_11_;
        float r_22_;
        float r_33_;

        float vel0 =400;


        // degの計算
        theta1_deg = ((theta1 * 180) / (M_PI));
        theta2_deg = ((theta2 * 180) / (M_PI));
        theta3_deg = ((theta3 * 180) / (M_PI));
        theta4_deg = ((theta4 * 180) / (M_PI));
        theta5_deg = ((theta5 * 180) / (M_PI));

        // RCLCPP_INFO(this->get_logger(), "theta1_deg:%f, theta2_deg:%f, theta3_deg:%f, theta4_deg:%f, theta5_deg:%f", theta1_deg, theta2_deg, theta3_deg, theta4_deg, theta5_deg);

        // 最大関節角度を考慮
        float off_theta1_deg_result = std::clamp(theta1_deg, deg_max_1 / 2, -deg_max_1 / 2); // 負から0
        float off_theta2_deg_result = std::clamp(theta2_deg, 0.0f, -deg_max_2); // 負から0
        float off_theta3_deg_result = std::clamp(theta3_deg, -deg_max_3, 0.0f); // 0から正
        float off_theta4_deg_result = std::clamp(theta4_deg, -30.0f, 165.0f); // 負から0
        float off_theta5_deg_result = std::clamp(theta5_deg, -deg_max_5 / 2, deg_max_5 / 2); // 0から正
        // RCLCPP_INFO(this->get_logger(), "1:%f, 2:%f, 3:%f, 4:%f, 5:%f", off_theta1_deg_result, off_theta2_deg_result, off_theta3_deg_result, off_theta4_deg_result, off_theta5_deg_result);

        // radに変更
        float theta1_rad = (off_theta1_deg_result * M_PI) / 180;
        float theta2_rad = (off_theta2_deg_result * M_PI) / 180;
        float theta3_rad = (off_theta3_deg_result * M_PI) / 180;
        float theta4_rad = (off_theta4_deg_result * M_PI) / 180;
        float theta5_rad = (off_theta5_deg_result * M_PI) / 180;

        // 順運動学の計算
        pos_x = cos(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        pos_y = sin(theta1_rad) * (L3 * cos(theta2_rad + theta3_rad) + L2 * cos(theta2_rad) + L0 + (L4 + L5) * sin(theta2_rad + theta3_rad + theta4_rad));
        pos_z = L3 * sin(theta2_rad + theta3_rad) + L2 * sin(theta2_rad) + (D + L1) - (L4 + L5) * cos(theta2_rad + theta3_rad + theta4_rad);

        r_11_ = cos(theta1_rad) * cos(theta2_rad + theta3_rad + theta4_rad) * cos(theta5_rad) + sin(theta1_rad) * sin(theta5_rad);
        r_22_ = -sin(theta1_rad) * cos(theta2_rad + theta3_rad + theta4_rad) * sin(theta5_rad) + cos(theta1_rad) * cos(theta5_rad);
        r_33_ = cos(theta2_rad + theta3_rad + theta4_rad);

        // RCLCPP_INFO(this->get_logger(), "pos_x:%f, pos_y:%f, pos_z:%f, r_11_:%f, r_22_:%f, r_33_:%f", pos_x, pos_y, pos_z, r_11_, r_22_, r_33_);

        // エンドエフェクタの誤差確認
        error_x = P_x - pos_x;
        error_y = P_y - pos_y;
        error_z = P_z - pos_z;
        // エンドエフェクタの単位行列の誤差
        error_r_11 = r_11 - r_11_;
        error_r_22 = r_22 - r_22_;
        error_r_33 = r_33 - r_33_;

        // 実機のオフセットに合わせる
        float off_theta1_deg = (deg_max_1 / 2) + off_theta1_deg_result;
        float off_theta2_deg = (-135) + off_theta2_deg_result;
        float off_theta3_deg = (160) + off_theta3_deg_result;
        float off_theta4_deg = (-75) + off_theta4_deg_result;
        float off_theta5_deg = (deg_max_5 / 2) + off_theta5_deg_result;
        // RCLCPP_INFO(this->get_logger(), "off_theta1_deg:%f, off_theta2_deg:%f, off_theta3_deg:%f, off_theta4_deg:%f, off_theta5_deg:%f", off_theta1_deg, off_theta2_deg, off_theta3_deg, off_theta4_deg, off_theta5_deg);

        // mradに変更
        // theta1_mrad = (off_theta1_deg_result * 1000 * M_PI) / 180;
        // theta2_mrad = (off_theta2_deg_result * 1000 * M_PI) / 180;
        // theta3_mrad = (off_theta3_deg_result * 1000 * M_PI) / 180;
        // theta4_mrad = (off_theta4_deg_result * 1000 * M_PI) / 180;
        // theta5_mrad = (off_theta4_deg_result * 1000 * M_PI) / 180;

        theta1_mrad = (off_theta1_deg * 1000 * M_PI) / 180;
        theta2_mrad = (off_theta2_deg * 1000 * M_PI) / 180;
        theta3_mrad = (off_theta3_deg * 1000 * M_PI) / 180;
        theta4_mrad = (off_theta4_deg * 1000 * M_PI) / 180;
        theta5_mrad = ((deg_max_5 / 2 )* 1000 * M_PI) / 180;

        auto goal = amir_interfaces::msg::AmirCmd();
        goal.angle[0] = theta1_mrad;
        goal.angle[1] = theta2_mrad;
        goal.angle[2] = theta3_mrad;
        goal.angle[3] = theta4_mrad;
        goal.angle[4] = theta5_mrad;    //現状0°に固定
        goal.angle[5] = current_angle6;
        goal.vel[0] = 200.0;
        goal.vel[1] = 200.0;
        goal.vel[2] = 200.0;
        goal.vel[3] = 200.0;
        goal.vel[4] = 400.0;
        goal.vel[5] = 200.0;
        // joint_pub_->publish(goal);
        RCLCPP_INFO(get_logger(),
        "Published [mrad]: %.1f,%.1f,%.1f,%.1f,%.1f,%.1f",
        goal.angle[0], goal.angle[1], goal.angle[2],
        goal.angle[3], goal.angle[4], goal.angle[5]);

  }

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<JointPublisher>();
  rclcpp::spin(node); 
  rclcpp::shutdown();
  return 0;
}