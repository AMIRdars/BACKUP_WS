#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>

// 同期用
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// OpenCV
#include <opencv2/opencv.hpp>

class VerifyConvertedPCL : public rclcpp::Node
{
public:
    VerifyConvertedPCL() : Node("verify_converted_pcl")
    {
        // QoS設定 (Reliable推奨)
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable();
        qos_reliable.durability_volatile();

        // トピック名
        // RGBはRealSenseから直接、PCLは自作の変換ノードから受け取る
        std::string rgb_topic = "/camera/camera/color/image_raw";
        std::string pcl_topic = "/converted_pointcloud"; 

        // message_filters設定
        rgb_sub_.subscribe(this, rgb_topic, qos_reliable.get_rmw_qos_profile());
        pcl_sub_.subscribe(this, pcl_topic, qos_reliable.get_rmw_qos_profile());

        // 同期ポリシー (ApproximateTime: 異なるノード経由なので多少の遅延ズレを許容する)
        sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(10), rgb_sub_, pcl_sub_
        );
        sync_->registerCallback(std::bind(&VerifyConvertedPCL::callback, this, std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "Verification Node Started.");
        RCLCPP_INFO(this->get_logger(), "Listening to: %s AND %s", rgb_topic.c_str(), pcl_topic.c_str());
    }

private:
    void callback(const sensor_msgs::msg::Image::ConstSharedPtr& rgb_msg,
                  const sensor_msgs::msg::PointCloud2::ConstSharedPtr& pcl_msg)
    {
        try {
            // 1. RGB画像をOpenCV形式に変換
            cv::Mat color_img = cv_bridge::toCvCopy(rgb_msg, "bgr8")->image;

            // 2. PointCloud2をPCL形式に変換
            // 前回の変換プログラムは XYZ のみを出力しているので PointXYZ を使う
            pcl::PointCloud<pcl::PointXYZ> cloud;
            pcl::fromROSMsg(*pcl_msg, cloud);

            // Ordered Cloudチェック (変換プログラムが正しければ height > 1 のはず)
            if (cloud.height <= 1) {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, 
                    "Received Unordered Cloud! Something is wrong with the converter.");
                return;
            }

            // 3. 中心座標の取得
            int center_x = color_img.cols / 2;
            int center_y = color_img.rows / 2;

            // 自作変換ノードは画像のピクセル並び順を維持しているため、座標(x,y)でアクセス可能
            pcl::PointXYZ pt = cloud(center_x, center_y);

            // 4. 情報表示
            std::string text = "NaN (Invalid)";
            cv::Scalar color(0, 0, 255); // Red

            // 値が有効かチェック
            if (std::isfinite(pt.x) && std::isfinite(pt.y) && std::isfinite(pt.z)) {
                std::stringstream ss;
                ss << "X:" << std::fixed << std::setprecision(3) << pt.x << " "
                   << "Y:" << std::fixed << std::setprecision(3) << pt.y << " "
                   << "Z:" << std::fixed << std::setprecision(3) << pt.z << "m";
                text = ss.str();
                color = cv::Scalar(0, 255, 0); // Green
            }

            // 5. 描画
            cv::circle(color_img, cv::Point(center_x, center_y), 5, color, -1);
            
            // 文字に黒い縁取りをつけて見やすくする
            cv::putText(color_img, text, cv::Point(center_x - 100, center_y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,0,0), 4);
            cv::putText(color_img, text, cv::Point(center_x - 100, center_y - 20),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);

            cv::imshow("Verification View", color_img);
            cv::waitKey(1);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
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
    auto node = std::make_shared<VerifyConvertedPCL>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}