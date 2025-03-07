#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

class PointCloudToGrid : public rclcpp::Node {
public:
    PointCloudToGrid() : Node("pointcloud_to_grid"), resolution_(0.2), width_(100), height_(100) {
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/groundgrid/segmented_cloud", 10, std::bind(&PointCloudToGrid::pointCloudCallback, this, std::placeholders::_1));
        
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
        occupancy_grid_.info.origin.position.z = 0.0;
        occupancy_grid_.info.origin.orientation.w = 1.0;
        occupancy_grid_.data.resize(width_ * height_, -1); // Initialize with unknown occupancy
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Reset grid
        std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);

        for (const auto& point : cloud.points) {
            if (point.intensity == 49.0) {  // Adjust intensity threshold as needed
                int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    occupancy_grid_.data[grid_y * width_ + grid_x] = 100; // Mark as occupied
                }
            }
        }

        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_pub_->publish(occupancy_grid_);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    double resolution_;
    int width_, height_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
