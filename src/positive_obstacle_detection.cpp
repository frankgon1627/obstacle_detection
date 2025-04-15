#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <opencv2/opencv.hpp>
#include <cmath>

using namespace std;

namespace obstacle_detection{

class PositiveObstacleDetectionNode : public rclcpp::Node {
public:
    PositiveObstacleDetectionNode(const rclcpp::NodeOptions & options) : Node("pointcloud_to_grid", options){
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 10, bind(&PositiveObstacleDetectionNode::odom_callback, this, placeholders::_1));
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/groundgrid/segmented_cloud", 10, bind(&PositiveObstacleDetectionNode::pointcloud_callback, this, placeholders::_1));

        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/positive_obstacle_grid", 10);
        dialated_occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/dialated_positive_obstacle_grid", 10);
        
        initializeOccupancyGrid();
    }

private:
    void initializeOccupancyGrid() {
        occupancy_grid_.header.frame_id = "odom";
        occupancy_grid_.info.resolution = resolution_;
        occupancy_grid_.info.width = width_;
        occupancy_grid_.info.height = height_;
        occupancy_grid_.info.origin.position.x = -static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = -static_cast<double>(height_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        occupancy_grid_.data.resize(width_ * height_, -1); 
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Reset grid
        fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);

        for (const auto& point : cloud.points) {
            // insensity value indicating non-ground point
            if (point.intensity == 99.0) { 
                // ignore any points that are too close to the Jackal
                if (sqrt(pow(point.x - odom_.pose.pose.position.x, 2) + pow(point.y - odom_.pose.pose.position.y, 2)) < 0.75) {
                    continue;
                }
                // ignore any points that are well above the Jackal
                if (point.z - odom_.pose.pose.position.z> 0.75){
                    continue;
                }

                int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    occupancy_grid_.data[grid_y * width_ + grid_x] = 100; 
                }
            }
        }

        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_pub_->publish(occupancy_grid_);
        RCLCPP_INFO(this->get_logger(), "Published Positive Occupancy Grid");

        nav_msgs::msg::OccupancyGrid dialated_occupancy_grid = dialate_occupancy_grid();
        dialated_occupancy_grid_pub_->publish(dialated_occupancy_grid);
        RCLCPP_INFO(this->get_logger(), "Published dialated Positive Occupancy Grid");
    }

    nav_msgs::msg::OccupancyGrid dialate_occupancy_grid(){
        // initialize relevant data
        nav_msgs::msg::OccupancyGrid dialated_occupancy_grid;
        dialated_occupancy_grid.header.stamp = occupancy_grid_.header.stamp;
        dialated_occupancy_grid.info = occupancy_grid_.info;
        dialated_occupancy_grid.data = occupancy_grid_.data;

        cv::Mat grid_map(height_, width_, CV_8UC1, const_cast<int8_t*>(dialated_occupancy_grid.data.data()));
        cv::Mat obstacle_mask = (grid_map == 100);

        int expansion_pixels = static_cast<int>(std::ceil(dialation_meters_ / resolution_));
        int kernel_size = 2 * expansion_pixels + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::Mat dilated_mask;
        cv::dilate(obstacle_mask, dilated_mask, kernel);

        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (dilated_mask.at<uint8_t>(y, x)) {
                    dialated_occupancy_grid.data[y * width_ + x] = 100;
                }
            }
        }

        return dialated_occupancy_grid;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dialated_occupancy_grid_pub_;

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;
    double dialation_meters_ = 0.75;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::PositiveObstacleDetectionNode)