// 修正箇所を含むコード全体
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>

class RgbPclOverlay : public rclcpp::Node
{
public:
    RgbPclOverlay() : Node("rgb_pcl_overlay")
    {
        // ---------------------------------------------------------
        // 【修正点】 QoSを Reliable (信頼性重視) に設定
        // RealSenseのPublisherがRELIABLEなので、それに合わせます
        // ---------------------------------------------------------
        rclcpp::QoS qos_settings(10); // KeepLast(10) と同等で、デフォルトは Reliable
        qos_settings.reliable();
        qos_settings.durability_volatile();

        // トピック名の設定
        std::string rgb_topic = "/camera/camera/color/image_raw";
        std::string pcl_topic = "/camera/camera/depth/color/points";

        RCLCPP_INFO(this->get_logger(), "Subscribing to:");
        RCLCPP_INFO(this->get_logger(), "  RGB: %s (Reliable)", rgb_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "  PCL: %s (Reliable)", pcl_topic.c_str());

        // message_filters サブスクライバーの初期化
        // 第3引数に修正した qos_settings を渡します
        rgb_sub_.subscribe(this, rgb_topic, qos_settings.get_rmw_qos_profile());
        pcl_sub_.subscribe(this, pcl_topic, qos_settings.get_rmw_qos_profile());

        // 同期ポリシーの設定
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, pcl_sub_
        );
        
        sync_->registerCallback(std::bind(&RgbPclOverlay::callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Node started. Waiting for sync...");
    }

    // ... (以下の private: 以降は変更なし) ...
private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl_msg)
    {
        // 動作確認用にログを出すと安心です
        RCLCPP_INFO_ONCE(this->get_logger(), "Frame Received! Processing...");

        try {
            cv::Mat color_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;
            pcl::PointCloud<pcl::PointXYZRGB> cloud;
            pcl::fromROSMsg(*pcl_msg, cloud);

            if (cloud.height <= 1) {
                cv::putText(color_img, "Error: PointCloud is UNORDERED!", cv::Point(20, 50), 
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 0, 255), 2);
            } else {
                int center_x = color_img.cols / 2;
                int center_y = color_img.rows / 2;
                pcl::PointXYZRGB point = cloud(center_x, center_y);

                std::string coord_text = "No Data";
                cv::Scalar text_color(0, 0, 255); 

                if (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z) && point.z != 0) {
                    std::stringstream ss;
                    ss << "X:" << std::fixed << std::setprecision(2) << point.x << " "
                       << "Y:" << std::fixed << std::setprecision(2) << point.y << " "
                       << "Z:" << std::fixed << std::setprecision(2) << point.z;
                    coord_text = ss.str();
                    text_color = cv::Scalar(0, 255, 0); 
                }
                cv::circle(color_img, cv::Point(center_x, center_y), 5, text_color, -1);
                cv::putText(color_img, coord_text, cv::Point(center_x - 100, center_y - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 4);
                cv::putText(color_img, coord_text, cv::Point(center_x - 100, center_y - 20),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7, text_color, 2);
            }
            cv::imshow("RGB-PCL Overlay", color_img);
            cv::waitKey(1);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception: %s", e.what());
        }
    }

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::PointCloud2> SyncPolicy;
    message_filters::Subscriber<sensor_msgs::msg::Image> rgb_sub_;
    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> pcl_sub_;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RgbPclOverlay>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}