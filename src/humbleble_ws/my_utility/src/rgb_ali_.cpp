#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <opencv2/opencv.hpp>

class AlignedDepthViewer : public rclcpp::Node
{
public:
    AlignedDepthViewer() : Node("aligned_depth_viewer")
    {
        // QoS設定 (前回の成功例に基づき Reliable に設定)
        rclcpp::QoS qos_settings(10);
        qos_settings.reliable();
        qos_settings.durability_volatile();

        // トピック名
        std::string rgb_topic = "/camera/camera/color/image_raw";
        std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";
        std::string info_topic = "/camera/camera/aligned_depth_to_color/camera_info";

        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  RGB: %s", rgb_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  Depth: %s", depth_topic.c_str());

        // 1. CameraInfoの取得（座標計算に必要）
        // これは一度だけ取れれば良いので、通常のサブスクライバーで受け取ります
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, qos_settings, 
            std::bind(&AlignedDepthViewer::info_callback, this, std::placeholders::_1));

        // 2. 画像と深度の同期サブスクライブ
        rgb_sub_.subscribe(this, rgb_topic, qos_settings.get_rmw_qos_profile());
        depth_sub_.subscribe(this, depth_topic, qos_settings.get_rmw_qos_profile());

        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, depth_sub_
        );
        sync_->registerCallback(std::bind(&AlignedDepthViewer::image_callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node started. Waiting for CameraInfo and Images...");
    }

private:
    // カメラ内部パラメータを受け取るコールバック
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!intrinsics_received_) {
            // Pinhole Camera Model Parameters
            // K = [fx, 0, ppx, 0, fy, ppy, 0, 0, 1]
            fx_ = msg->k[0];
            ppx_ = msg->k[2];
            fy_ = msg->k[4];
            ppy_ = msg->k[5];
            
            intrinsics_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Camera Intrinsics Loaded: fx=%.2f, fy=%.2f, ppx=%.2f, ppy=%.2f", fx_, fy_, ppx_, ppy_);
            
            // 必要なくなったらサブスクライブを解除しても良いですが、ここではそのままにします
        }
    }

    // 同期された画像を受け取るコールバック
    void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                        const sensor_msgs::msg::Image::ConstSharedPtr& depth_msg)
    {
        if (!intrinsics_received_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for Camera Info...");
            return;
        }

        try {
            // RGB画像の変換
            cv::Mat color_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
            
            // 深度画像の変換 (RealSenseのDepthは通常 16-bit unsigned int, 単位はミリメートル)
            cv::Mat depth_img = cv_bridge::toCvCopy(depth_msg, "16UC1")->image;

            int center_x = color_img.cols / 2;
            int center_y = color_img.rows / 2;

            // 中心ピクセルの深度値を取得 (mm単位)
            uint16_t depth_mm = depth_img.at<uint16_t>(center_y, center_x);

            std::string text = "Z: 0.000 (No Data)";
            cv::Scalar text_color(0, 0, 255); // Red

            // 0 は計測不能（無効値）
            if (depth_mm > 0) {
                // mm -> meters
                float z_meters = static_cast<float>(depth_mm) / 1000.0f;

                // 2Dピクセル座標 -> 3D空間座標への変換公式
                // X = (u - cx) * Z / fx
                // Y = (v - cy) * Z / fy
                float x_meters = (center_x - ppx_) * z_meters / fx_;
                float y_meters = (center_y - ppy_) * z_meters / fy_;

                std::stringstream ss;
                ss << "X:" << std::fixed << std::setprecision(3) << x_meters << " "
                   << "Y:" << std::fixed << std::setprecision(3) << y_meters << " "
                   << "Z:" << std::fixed << std::setprecision(3) << z_meters << "m";
                
                text = ss.str();
                text_color = cv::Scalar(0, 255, 0); // Green
                
                // コンソールにも出力して確認しやすくする
                // RCLCPP_INFO(this->get_logger(), "Center: %s", text.c_str());
            }

            // 描画
            cv::circle(color_img, cv::Point(center_x, center_y), 5, text_color, -1);
            
            // 文字が見やすいように黒縁をつける
            cv::putText(color_img, text, cv::Point(center_x - 100, center_y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 4);
            cv::putText(color_img, text, cv::Point(center_x - 100, center_y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);

            cv::imshow("Aligned Depth View", color_img);
            cv::waitKey(1);

        } catch (const cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::Image> depth_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;

    bool intrinsics_received_ = false;
    double fx_ = 0, fy_ = 0, ppx_ = 0, ppy_ = 0;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AlignedDepthViewer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}