#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2/exceptions.h"
#include "future"

using namespace std;
using PointArray = array<array<array<float, 3>, 1024>, 64>;
using FeaturePoints = vector<array<float, 7>>;
constexpr size_t vertical_channels_ = 64;
constexpr size_t horizontal_channels_ = 1024;

namespace obstacle_detection{

class CombinerNode : public rclcpp::Node
{
public:
    CombinerNode(const rclcpp::NodeOptions & options) : 
    Node("combiner", options){
        dilated_positive_obstacle_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/dilated_positive_obstacle_grid", 10, bind(&CombinerNode::dilated_positive_obstacle_grid_callback, this, placeholders::_1));
        filtered_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/filtered_risk_map", 10, bind(&CombinerNode::filtered_grid_callback, this, placeholders::_1));
        combined_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacle_detection/combined_map", 10);

        RCLCPP_INFO(this->get_logger(), "Combiner Node Initialized");
    }

private:
    void dilated_positive_obstacle_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        dilated_positive_obstacle_grid_ = msg;
        height_ = dilated_positive_obstacle_grid_->info.height;
        width_ = dilated_positive_obstacle_grid_->info.width;
        map_resolution_ = dilated_positive_obstacle_grid_->info.resolution;
    }

    void filtered_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        filtered_grid_ = msg;

        if (!dilated_positive_obstacle_grid_)
        {
            RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
            return;
        }

        // extract relevant Occupancy Grid Information
        nav_msgs::msg::OccupancyGrid combined_map;
        combined_map.header = dilated_positive_obstacle_grid_->header;
        combined_map.info = dilated_positive_obstacle_grid_->info;;
        combined_map.data = dilated_positive_obstacle_grid_->data;

        // make combined risk map
        for(size_t i=0; i < filtered_grid_->data.size(); i++){
            if (combined_map.data[i] == 0){
                combined_map.data[i] = filtered_grid_->data[i];
            }
        }
        if (combined_map_pub_->get_subscription_count() > 0){
            combined_map_pub_->publish(combined_map);
        }
        RCLCPP_INFO(this->get_logger(), "Published Combined Risk Map.");
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dilated_positive_obstacle_grid_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr filtered_grid_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr combined_map_pub_;

    // Obstacle map variables
    nav_msgs::msg::OccupancyGrid::SharedPtr dilated_positive_obstacle_grid_;
    nav_msgs::msg::OccupancyGrid::SharedPtr filtered_grid_;
    int height_;
    int width_;
    double map_resolution_;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::CombinerNode)
