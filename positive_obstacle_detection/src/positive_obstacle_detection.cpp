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

class PointCloudToGrid : public rclcpp::Node {
public:
    PointCloudToGrid() : Node("pointcloud_to_grid"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 10, std::bind(&PointCloudToGrid::odomCallback, this, std::placeholders::_1));
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

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Reset grid
        std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);

        geometry_msgs::msg::TransformStamped transform_stamped;
        try {
            transform_stamped = tf_buffer_.lookupTransform("map", "os_sensor", tf2::TimePointZero);
        } catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform from os_sensor to map: %s", ex.what());
            return;
        }

        for (const auto& point : cloud.points) {
            if (point.intensity == 99.0) {  // Adjust intensity threshold as needed
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.frame_id = msg->header.frame_id;
                point_in.point.x = point.x;
                point_in.point.y = point.y;
                point_in.point.z = point.z;

                tf2::doTransform(point_in, point_out, transform_stamped);

                int grid_x = static_cast<int>((point_out.point.x - occupancy_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point_out.point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    occupancy_grid_.data[grid_y * width_ + grid_x] = 100; 
                }
            }
        }

        occupancy_grid_.header.stamp = this->now();
        occupancy_grid_pub_->publish(occupancy_grid_);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::OccupancyGrid occupancy_grid_;
    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
