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
#include <unordered_set>

class PointCloudToGrid : public rclcpp::Node {
public:
    PointCloudToGrid() : Node("pointcloud_to_grid"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_) {
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/dlio/odom_node/odom", 10, std::bind(&PointCloudToGrid::odom_callback, this, std::placeholders::_1));
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/groundgrid/segmented_cloud", 10, std::bind(&PointCloudToGrid::pointcloud_callback, this, std::placeholders::_1));
        
        occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/positive_obstacle_grid", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&PointCloudToGrid::transform_callback, this));
        
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

    void transform_callback(){
        try{
            lidar_to_map_transform_ = tf_buffer_.lookupTransform("os_sensor", "map", tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        occupancy_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        occupancy_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "Got Segmented Pointcloud");
        std::unordered_set<float> unique_intensities;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // Reset grid
        std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), 0);

        for (const auto& point : cloud.points) {
            unique_intensities.insert(point.intensity);
            if (point.intensity == 99.0) { 
                geometry_msgs::msg::PointStamped point_in, point_out;
                point_in.header.frame_id = msg->header.frame_id;
                point_in.point.x = point.x;
                point_in.point.y = point.y;
                point_in.point.z = point.z;

                tf2::doTransform(point_in, point_out, lidar_to_map_transform_);

                int grid_x = static_cast<int>((point_out.point.x - occupancy_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point_out.point.y - occupancy_grid_.info.origin.position.y) / resolution_);
                
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    occupancy_grid_.data[grid_y * width_ + grid_x] = 100; 
                }
            }
        }
        RCLCPP_INFO(this->get_logger(), "Unique intensities:");
        for (const float& intensity : unique_intensities) {
            RCLCPP_INFO(this->get_logger(), "Intensity: %f", intensity);
    }

        RCLCPP_INFO(this->get_logger(), "Going to Publish Grid");
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

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::TransformStamped lidar_to_map_transform_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointCloudToGrid>());
    rclcpp::shutdown();
    return 0;
}
