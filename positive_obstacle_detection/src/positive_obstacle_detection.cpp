// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/point_cloud2.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <nav_msgs/msg/occupancy_grid.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <pcl_conversions/pcl_conversions.h>
// #include <pcl/point_cloud.h>
// #include <pcl/point_types.h>
// #include <tf2_ros/transform_listener.h>
// #include <tf2_ros/buffer.h>
// #include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// using namespace std;

// class PointCloudToGrid : public rclcpp::Node {
// public:
//     PointCloudToGrid() : Node("pointcloud_to_grid") {
//         odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
//             "/dlio/odom_node/odom", 10, bind(&PointCloudToGrid::odom_callback, this, placeholders::_1));
//         pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
//             "/groundgrid/segmented_cloud", 10, bind(&PointCloudToGrid::pointcloud_callback, this, placeholders::_1));
        
//         occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
//             "/obstacle_detection/positive_obstacle_grid", 10);
        
//         initializeOccupancyGrid();
//     }

// private:
//     void initializeOccupancyGrid() {
//         occupancy_grid_.header.frame_id = "map";
//         occupancy_grid_.info.resolution = resolution_;
//         occupancy_grid_.info.width = width_;
//         occupancy_grid_.info.height = height_;
//         occupancy_grid_.info.origin.position.x = -static_cast<double>(width_) * resolution_ / 2.0;
//         occupancy_grid_.info.origin.position.y = -static_cast<double>(height_) * resolution_ / 2.0;
//         occupancy_grid_.info.origin.position.z = 0.0;
//         occupancy_grid_.info.origin.orientation.w = 1.0;
//         occupancy_grid_.data.resize(width_ * height_, -1); // Initialize with unknown occupancy
//     }

//     void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
//         odom_ = *msg;
//         occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
//         occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
//         RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
//     }

//     void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
//         pcl::PointCloud<pcl::PointXYZI> cloud;
//         pcl::fromROSMsg(*msg, cloud);

//         // Reset grid
//         fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);

//         for (const auto& point : cloud.points) {
//             // insensity value indicating non-ground point
//             if (point.intensity == 99.0) { 
//                 // ignore any points that are too close to the Jackal
//                 if (sqrt(pow(point.x - odom_.pose.pose.position.x, 2) + pow(point.y - odom_.pose.pose.position.y, 2)) < 0.75) {
//                     continue;
//                 }
//                 // ignore any points that are well above the Jackal
//                 if (point.z > 0.75){
//                     continue;
//                 }

//                 int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / resolution_);
//                 int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
//                 if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
//                     occupancy_grid_.data[grid_y * width_ + grid_x] = 100; 
//                 }
//             }
//         }

//         occupancy_grid_.header.stamp = this->now();
//         occupancy_grid_pub_->publish(occupancy_grid_);
//         RCLCPP_INFO(this->get_logger(), "Published Positive Occupancy Grid");
//     }

//     rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
//     rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
//     rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

//     nav_msgs::msg::Odometry odom_;
//     nav_msgs::msg::OccupancyGrid occupancy_grid_;
//     double resolution_ = 0.2;
//     int width_ = 100;
//     int height_ = 100;
// };

// int main(int argc, char **argv) {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<PointCloudToGrid>());
//     rclcpp::shutdown();
//     return 0;
// }

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <vector>
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
            "/obstacle_detection/positive_obstacle_grid", 10);
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
        occupancy_grid_.info.origin.orientation.w = 1.0;
        
        int grid_size = width_ * height_;
        occupancy_grid_.data.resize(grid_size, 0); 
        log_odds_.resize(grid_size, 0.0);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);
        updateOccupancyGrid(cloud);
    }

    void updateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZI>& cloud) {
        for (double &log_odd : log_odds_) {
            log_odd *= decay_factor_;
        }

        int robot_x = static_cast<int>((odom_.pose.pose.position.x - occupancy_grid_.info.origin.position.x) / resolution_);
        int robot_y = static_cast<int>((odom_.pose.pose.position.y - occupancy_grid_.info.origin.position.y) / resolution_);
        
        for (const auto& point : cloud.points) {
            if (point.intensity != 99.0) continue;
            if (point.z > 0.75) continue;
            
            int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / resolution_);
            int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / resolution_);
            
            if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                updateFreeSpace(robot_x, robot_y, grid_x, grid_y);
                int index = grid_y * width_ + grid_x;
                log_odds_[index] += log_odds_occ_ - log_odds_prior_;
                log_odds_[index] = std::clamp(log_odds_[index], min_log_odds_, max_log_odds_);
                occupancy_grid_.data[index] = probabilityFromLogOdds(log_odds_[index]);
            }
        }
        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_pub_->publish(occupancy_grid_);
    }

    void updateFreeSpace(int x0, int y0, int x1, int y1) {
        int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
        int dy = abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
        int err = (dx > dy ? dx : -dy) / 2, e2;
        
        while (x0 != x1 || y0 != y1) {
            int index = y0 * width_ + x0;
            log_odds_[index] += log_odds_free_ - log_odds_prior_;
            log_odds_[index] = std::clamp(log_odds_[index], min_log_odds_, max_log_odds_);
            occupancy_grid_.data[index] = probabilityFromLogOdds(log_odds_[index]);
            e2 = err;
            if (e2 > -dx) { err -= dy; x0 += sx; }
            if (e2 < dy) { err += dx; y0 += sy; }
        }
    }

    int probabilityFromLogOdds(double log_odds) {
        return static_cast<int>(100.0 * (1.0 - 1.0 / (1.0 + exp(log_odds))));
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    
    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    vector<double> log_odds_;

    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;
    double log_odds_prior_ = 0.0;
    double log_odds_occ_ = 2.0;
    double log_odds_free_ = -1.5;
    double min_log_odds_ = -5.0;
    double max_log_odds_ = 5.0;
    double decay_factor_ = 0.95;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
