#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vector>
#include <algorithm>
#include <cmath>

using namespace std;

class PointCloudToGrid : public rclcpp::Node {
public:
    PointCloudToGrid() : Node("pointcloud_to_grid") {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 10, bind(&PointCloudToGrid::odom_callback, this, placeholders::_1));
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/groundgrid/segmented_cloud", 10, bind(&PointCloudToGrid::pointcloud_callback, this, placeholders::_1)); 
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/positive_obstacle_grid1", 10);
        
        log_odds_.resize(width_ * height_, 0.0);
        initializeOccupancyGrid();
    }

private:
    void initializeOccupancyGrid() {
        occupancy_grid_.header.frame_id = "map";
        occupancy_grid_.info.resolution = resolution_;
        occupancy_grid_.info.width = width_;
        occupancy_grid_.info.height = height_;
        occupancy_grid_.info.origin.position.x = -static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = -static_cast<double>(height_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        occupancy_grid_.data.resize(width_ * height_, -1);
        fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
    }

    void raycast(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = dx + dy, e2;
        while (true) {
            int index = y0 * width_ + x0;
            if (index >= 0 && index < log_odds_.size()) {
                log_odds_[index] += free_log_odds_;
            }
            if (x0 == x1 && y0 == y1) break;
            e2 = 2 * err;
            if (e2 >= dy) { err += dy; x0 += sx; }
            if (e2 <= dx) { err += dx; y0 += sy; }
        }
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        for (double& log_odds : log_odds_) {
            log_odds = log_odds * decay_factor_ + decay_bias_;
        }

        int robot_x = static_cast<int>((odom_.pose.pose.position.x - occupancy_grid_.info.origin.position.x) / resolution_);
        int robot_y = static_cast<int>((odom_.pose.pose.position.y - occupancy_grid_.info.origin.position.y) / resolution_);

        // for (size_t i = 0; i < log_odds_.size(); ++i) {
        //     int x = i % width_;
        //     int y = i / width_;
        //     double world_x = occupancy_grid_.info.origin.position.x + x * resolution_;
        //     double world_y = occupancy_grid_.info.origin.position.y + y * resolution_;
        //     double distance = sqrt(pow(world_x - odom_.pose.pose.position.x, 2) + pow(world_y - odom_.pose.pose.position.y, 2));
        //     if (distance > clearing_radius_) {
        //         occupancy_grid_.data[i] = -1;
        //         log_odds_[i] = 0.0;
        //     }
        // }

        for (const auto& point : cloud.points) {
            if (point.intensity == 99.0) {
                if (sqrt(pow(point.x - odom_.pose.pose.position.x, 2) + pow(point.y - odom_.pose.pose.position.y, 2)) < 0.75) {
                    continue;
                }
                if (point.z > 0.75) {
                    continue;
                }

                int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    int index = grid_y * width_ + grid_x;
                    log_odds_[index] += occupied_log_odds_;
                    raycast(robot_x, robot_y, grid_x, grid_y);
                }
            }
        }

        for (size_t i = 0; i < log_odds_.size(); ++i) {
            double probability = 1.0 - 1.0 / (1.0 + exp(log_odds_[i]));
            occupancy_grid_.data[i] = static_cast<int>(probability * 100);
        }

        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_pub_->publish(occupancy_grid_);
        RCLCPP_INFO(this->get_logger(), "Published Positive Occupancy Grid");
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    std::vector<double> log_odds_;
    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;
    double decay_factor_ = 0.95;
    double decay_bias_ = -0.05;
    double occupied_log_odds_ = 0.7;
    double free_log_odds_ = -0.4;
    double clearing_radius_ = 5.0;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
