#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// メッセージ生成用
#include <sensor_msgs/point_cloud2_iterator.hpp>

class DepthToPointCloud : public rclcpp::Node
{
public:
    DepthToPointCloud() : Node("depth_to_pcl_converter")
    {
        // QoS設定 (RealSenseに合わせてReliable)
        rclcpp::QoS qos_reliable(10);
        qos_reliable.reliable();
        qos_reliable.durability_volatile();

        // トピック名
        std::string depth_topic = "/camera/camera/aligned_depth_to_color/image_raw";
        std::string info_topic = "/camera/camera/aligned_depth_to_color/camera_info";
        std::string pub_topic = "/converted_pointcloud"; // 生成される点群のトピック名

        // 1. CameraInfoの取得 (一度だけ取得)
        info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            info_topic, qos_reliable,
            std::bind(&DepthToPointCloud::info_callback, this, std::placeholders::_1));

        // 2. 深度画像の購読
        depth_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            depth_topic, qos_reliable,
            std::bind(&DepthToPointCloud::depth_callback, this, std::placeholders::_1));

        // 3. ポイントクラウドの発行
        pcl_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(pub_topic, qos_reliable);

        RCLCPP_INFO(this->get_logger(), "Converter Node Started.");
        RCLCPP_INFO(this->get_logger(), "Input Depth: %s", depth_topic.c_str());
        RCLCPP_INFO(this->get_logger(), "Output Cloud: %s", pub_topic.c_str());
    }

private:
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if (!intrinsics_received_) {
            // カメラ内部パラメータの保存
            // K = [fx, 0, cx, 0, fy, cy, 0, 0, 1]
            fx_ = msg->k[0];
            cx_ = msg->k[2];
            fy_ = msg->k[4];
            cy_ = msg->k[5];
            frame_id_ = msg->header.frame_id; // depth_optical_frame等
            intrinsics_received_ = true;
            RCLCPP_INFO(this->get_logger(), "Intrinsics Set: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx_, fy_, cx_, cy_);
        }
    }

    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        if (!intrinsics_received_) return;

        // 画像をOpenCV形式に変換 (16bit unsigned int: mm単位)
        cv_bridge::CvImagePtr cv_ptr;
        try {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
        } catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        // PointCloud2メッセージの作成
        auto cloud_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        cloud_msg->header = msg->header; // タイムスタンプ等を継承
        cloud_msg->height = msg->height;
        cloud_msg->width = msg->width;
        cloud_msg->is_dense = false; // 無効な点(NaN)が含まれる可能性があるためfalse
        cloud_msg->is_bigendian = false;

        // フィールドの設定 (x, y, z)
        sensor_msgs::PointCloud2Modifier modifier(*cloud_msg);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(msg->height * msg->width);

        cloud_msg->height = msg->height;
        cloud_msg->width = msg->width;
        cloud_msg->row_step = cloud_msg->width * cloud_msg->point_step; // 行ごとのバイト数も更新

        // イテレータの準備 (データを書き込むためのポインタ)
        sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");

        const cv::Mat& depth_img = cv_ptr->image;

        // 全ピクセルを走査して3次元座標へ変換
        for (int v = 0; v < depth_img.rows; ++v)
        {
            for (int u = 0; u < depth_img.cols; ++u)
            {
                // 深度値 (mm)
                uint16_t depth_raw = depth_img.at<uint16_t>(v, u);

                if (depth_raw > 0) {
                    // メートル単位に変換
                    float z = static_cast<float>(depth_raw) / 1000.0f;
                    
                    // Pinhole Camera Model 逆投影
                    float x = (static_cast<float>(u) - cx_) * z / fx_;
                    float y = (static_cast<float>(v) - cy_) * z / fy_;

                    *iter_x = x;
                    *iter_y = y;
                    *iter_z = z;
                } else {
                    // 無効なデータは NaN (Not a Number) に設定
                    // OctomapなどはNaNを無視してくれる
                    float bad_point = std::numeric_limits<float>::quiet_NaN();
                    *iter_x = bad_point;
                    *iter_y = bad_point;
                    *iter_z = bad_point;
                }

                // イテレータを進める
                ++iter_x;
                ++iter_y;
                ++iter_z;
            }
        }

        // 発行
        pcl_pub_->publish(*cloud_msg);
    }

    // メンバ変数
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_pub_;

    bool intrinsics_received_ = false;
    double fx_, fy_, cx_, cy_;
    std::string frame_id_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DepthToPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}