#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "obstacle_detection_msgs/msg/risk_map.hpp"
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
        blurred_grid_subscriber_ = this->create_subscription<obstacle_detection_msgs::msg::RiskMap>(
            "/obstacle_detection/blurred_risk_map", 10, bind(&CombinerNode::blurred_grid_callback, this, placeholders::_1));
        blurred_grid_rviz_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/blurred_risk_map_rviz", 10, bind(&CombinerNode::blurred_grid_rviz_callback, this, placeholders::_1));

        combined_map_pub_ = this->create_publisher<obstacle_detection_msgs::msg::RiskMap>("/obstacle_detection/combined_map", 10);
        combined_map_rviz_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/obstacle_detection/combined_map_rviz", 10);

        RCLCPP_INFO(this->get_logger(), "Combiner Node Initialized");
    }

private:
    void dilated_positive_obstacle_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        dilated_positive_obstacle_grid_ = msg;
        height_ = dilated_positive_obstacle_grid_->info.height;
        width_ = dilated_positive_obstacle_grid_->info.width;
        map_resolution_ = dilated_positive_obstacle_grid_->info.resolution;
        dilated_positive_origin_ = dilated_positive_obstacle_grid_->info.origin;
    }

    void blurred_grid_callback(const obstacle_detection_msgs::msg::RiskMap::SharedPtr msg){
        obstacle_detection_msgs::msg::RiskMap::SharedPtr blurred_grid = msg;

        if (!dilated_positive_obstacle_grid_)
        {
            RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
            return;
        }

        // extract relevant Occupancy Grid Information
        obstacle_detection_msgs::msg::RiskMap combined_map;
        combined_map.header = blurred_grid->header;
        combined_map.info = blurred_grid->info;
        combined_map.data = vector<float>(width_ * height_, 0);
        double combined_map_origin_x = combined_map.info.origin.position.x;
        double combined_map_origin_y = combined_map.info.origin.position.y;

        // make combined risk map
        for(size_t cell_index=0; cell_index < combined_map.data.size(); cell_index++){
            // get the 2D position of the current cell of the combined grid
            int cell_i = cell_index % width_;
            int cell_j = cell_index / height_;  
            double cell_x = combined_map_origin_x + (static_cast<double>(cell_i) + 0.5) * map_resolution_;
            double cell_y = combined_map_origin_y + (static_cast<double>(cell_j) + 0.5) * map_resolution_;

            // get the cell value of the dilated positive grid at this 2D location
            int pos_grid_j = int((cell_x - dilated_positive_origin_.position.x) / map_resolution_);
            int pos_grid_i = int((cell_y - dilated_positive_origin_.position.y) / map_resolution_);
            int flattened_index = pos_grid_i * width_ + pos_grid_j;

            if(0 <= pos_grid_i && pos_grid_i < height_ && 0 <= pos_grid_j && pos_grid_j < width_){
                int flattened_index = pos_grid_i * width_ + pos_grid_j;
                if (dilated_positive_obstacle_grid_->data[flattened_index] == 100){
                    combined_map.data[cell_index] = 100;
                }
            }
            else{
                combined_map.data[cell_index] = blurred_grid->data[cell_index];
            }
        }

        if (combined_map_pub_->get_subscription_count() > 0){
            combined_map_pub_->publish(combined_map);
        }
        RCLCPP_INFO(this->get_logger(), "Published Combined Risk Map.");
    }

    void blurred_grid_rviz_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        nav_msgs::msg::OccupancyGrid::SharedPtr blurred_grid_rviz = msg;

        if (!dilated_positive_obstacle_grid_)
        {
            RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
            return;
        }

        // extract relevant Occupancy Grid Information
        nav_msgs::msg::OccupancyGrid combined_map_rviz;
        combined_map_rviz.header = blurred_grid_rviz->header;
        combined_map_rviz.info = blurred_grid_rviz->info;
        combined_map_rviz.data = vector<int8_t>(width_ * height_, 0);
        double combined_map_origin_x = combined_map_rviz.info.origin.position.x;
        double combined_map_origin_y = combined_map_rviz.info.origin.position.y;

        // make combined risk map
        for(size_t cell_index=0; cell_index < combined_map_rviz.data.size(); cell_index++){
            // get the 2D position of the current cell of the combined grid
            int cell_i = cell_index % width_;
            int cell_j = cell_index / height_;  
            double cell_x = combined_map_origin_x + (static_cast<double>(cell_i) + 0.5) * map_resolution_;
            double cell_y = combined_map_origin_y + (static_cast<double>(cell_j) + 0.5) * map_resolution_;

            // get the cell value of the dilated positive grid at this 2D location
            int pos_grid_j = int((cell_x - dilated_positive_origin_.position.x) / map_resolution_);
            int pos_grid_i = int((cell_y - dilated_positive_origin_.position.y) / map_resolution_);

            if(0 <= pos_grid_i && pos_grid_i < height_ && 0 <= pos_grid_j && pos_grid_j < width_){
                int flattened_index = pos_grid_i * width_ + pos_grid_j;
                if (dilated_positive_obstacle_grid_->data[flattened_index] == 100){
                    combined_map_rviz.data[cell_index] = 100;
                }
            }
            else{
                combined_map_rviz.data[cell_index] = blurred_grid_rviz->data[cell_index];
            }
        }

        if (combined_map_rviz_pub_->get_subscription_count() > 0){
            combined_map_rviz_pub_->publish(combined_map_rviz);
        }
        RCLCPP_INFO(this->get_logger(), "Published Combined Risk Map.");
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr dilated_positive_obstacle_grid_subscriber_;
    rclcpp::Subscription<obstacle_detection_msgs::msg::RiskMap>::SharedPtr blurred_grid_subscriber_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr blurred_grid_rviz_subscriber_;
    rclcpp::Publisher<obstacle_detection_msgs::msg::RiskMap>::SharedPtr combined_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr combined_map_rviz_pub_;

    // Obstacle map variables
    nav_msgs::msg::OccupancyGrid::SharedPtr dilated_positive_obstacle_grid_;
    int height_;
    int width_;
    double map_resolution_;
    geometry_msgs::msg::Pose dilated_positive_origin_;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::CombinerNode)
