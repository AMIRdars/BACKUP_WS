#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include "nav2_costmap_2d/layered_costmap.hpp"
#include <math.h>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("set_cost_around_point");
    auto costmap_ros = std::make_shared<nav2_costmap_2d::Costmap2DROS>("global_costmap");
    // costmap_ros->configure();
    costmap_ros->on_configure(rclcpp_lifecycle::State());
    // while(!costmap_ros->getCostmap()->isInitialized()){
    //     std::this_thread::sleep_for(std::chrono::milliseconds(60));
    // }
    RCLCPP_INFO(node->get_logger(), "Generated path1:");

    // costmap_ros->waitForStaticMap();

    // set the center point and radius
    // double radius = 0.3;
    // RCLCPP_INFO(node->get_logger(), "Generated path2:");

    // convert world coordinates to map coordinates
    unsigned int mx, my;
    costmap_ros->getCostmap()->worldToMap(0, 0, mx, my);
    RCLCPP_INFO(node->get_logger(), "Generated path3:");
    RCLCPP_INFO(node->get_logger(), "Generated pathx: %d", mx);
    RCLCPP_INFO(node->get_logger(), "Generated pathy: %d", my);

    // set the cost of the center point to 0
    costmap_ros->getCostmap()->setCost(mx, my, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+1, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+2, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+3, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+4, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+5, 0);
    // costmap_ros->getCostmap()->setCost(mx, my+6, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+1, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+2, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+3, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+4, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+5, 0);
    // costmap_ros->getCostmap()->setCost(mx+1, my+6, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+1, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+2, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+3, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+4, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+5, 0);
    // costmap_ros->getCostmap()->setCost(mx+2, my+6, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+1, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+2, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+3, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+4, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+5, 0);
    // costmap_ros->getCostmap()->setCost(mx+3, my+6, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+1, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+2, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+3, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+4, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+5, 0);
    // costmap_ros->getCostmap()->setCost(mx+4, my+6, 0);




    RCLCPP_INFO(node->get_logger(), "Generated path4:");

    // double resolution = costmap_ros->getCostmap()->getResolution();
    // int radius_pixel = (int)(radius / resolution);
    // RCLCPP_INFO(node->get_logger(), "Generated path5:");
    
    //6グリッド=0.5[m]
    for(int x = 0; x <= 6; x++){
        // RCLCPP_INFO(node->get_logger(), "Generated path6:");
        for(int y = 0; y <= 6; y++){
            // RCLCPP_INFO(node->get_logger(), "Generated path7:");
            // double dist = sqrt(x*x + y*y) * resolution;
            // if(dist <= radius){
            // RCLCPP_INFO(node->get_logger(), "Generated path8:");
            RCLCPP_INFO(node->get_logger(), "Generated pathx: %d", x);
            RCLCPP_INFO(node->get_logger(), "Generated pathy: %d", y);
            int xx = mx + x;
            int yy = my + y;

            costmap_ros->getCostmap()->setCost(xx, yy, 0);
            // }
        }
    }

    // set the cost of the points within the radius to 0
    // for (int i = -radius/costmap_ros->getCostmap()->getResolution(); i <= radius/costmap_ros->getCostmap()->getResolution(); i++) {
    //     RCLCPP_INFO(node->get_logger(), "Generated path5:");
    //     for (int j = -radius/costmap_ros->getCostmap()->getResolution(); j <= radius/costmap_ros->getCostmap()->getResolution(); j++) {
    //         RCLCPP_INFO(node->get_logger(), "Generated path6:");
    //         if (sqrt(pow(i*costmap_ros->getCostmap()->getResolution(), 2) + pow(j*costmap_ros->getCostmap()->getResolution(), 2)) <= radius) {
    //             costmap_ros->getCostmap()->setCost(mx + i, my + j, 0);
    //         }
    //     }
    // }
    RCLCPP_INFO(node->get_logger(), "Successfully set costmap cost around point");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
