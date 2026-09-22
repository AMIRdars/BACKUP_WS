#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>

class PcToOctomapNode : public rclcpp::Node
{
public:
  PcToOctomapNode()
  : Node("pc_to_octomap_node"),
    octree_(0.05) // OctoMapの空間分解能（ボクセルサイズ: メートル単位）を指定
  {
    // PointCloud2を受信するサブスクライバー
    sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/converted_pointcloud", 10,
      std::bind(&PcToOctomapNode::pointcloudCallback, this, std::placeholders::_1));

    // OctoMapを配信するパブリッシャー
    pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("octomap_out", 10);
    
    RCLCPP_INFO(this->get_logger(), "PcToOctomapNode has started.");
  }

private:
  void pointcloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    octomap::Pointcloud octo_cloud;

    // PointCloud2Iteratorを使用してバイナリデータからX, Y, Zの座標値を直接抽出
    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
      // NaN値や無限大が含まれるデータを除外
      if (!std::isnan(*iter_x) && !std::isnan(*iter_y) && !std::isnan(*iter_z)) {
        // octo_cloud.push_back(*iter_x, *iter_y, *iter_z);

        // 変数に格納してコードの見通しを良くする
        float x = *iter_x;
        float y = *iter_y;
        float z = *iter_z;

        // パターンA: 厳密な「ボックス（直方体）」としてX, Y, Zが全て±2.5m以内の場合を抽出
        if (std::abs(x) <= 2.5f && std::abs(y) <= 2.5f && std::abs(z) <= 2.5f) {
          octo_cloud.push_back(x, y, z);
        }

        /* // パターンB: センサからの「直線距離（球体）」が2.5m以内の場合を抽出したい時はこちらを使用
        // 距離の2乗（2.5 * 2.5 = 6.25）で比較し、平方根の計算コストを削減します
        // if ((x*x + y*y + z*z) <= 6.25f) {
        //   octo_cloud.push_back(x, y, z);
        // }
        */

      }
    }

    // 抽出した点群をOcTreeに挿入
    // ※実運用ではセンサの位置や姿勢(TF)を加味し、マップ座標系へ変換した上で処理することが推奨されます
    octomap::point3d sensor_origin(0.0, 0.0, 0.0);
    octree_.insertPointCloud(octo_cloud, sensor_origin);

    // C++のデータ構造からROS 2のメッセージ構造へ変換
    octomap_msgs::msg::Octomap map_msg;
    map_msg.header = msg->header; // フレームIDやタイムスタンプを継承
    
    // binaryMapToMsgを使用してバイナリ形式にシリアライズ（Free/Occupiedのみで通信帯域を節約）
    if (octomap_msgs::binaryMapToMsg(octree_, map_msg)) {
      pub_->publish(map_msg);
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to serialize OctoMap.");
    }
  }

  octomap::OcTree octree_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
  rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr pub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PcToOctomapNode>());
  rclcpp::shutdown();
  return 0;
}