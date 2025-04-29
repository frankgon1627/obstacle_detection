#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
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

        positive_obstacle_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/positive_obstacle_grid", 10);
        dilated_positive_obstacle_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/dilated_positive_obstacle_grid", 10);
        
        initializeOccupancyGrid();
    }

private:
    void initializeOccupancyGrid() {
        positive_obstacle_grid_.header.frame_id = "odom";
        positive_obstacle_grid_.info.resolution = resolution_;
        positive_obstacle_grid_.info.width = width_;
        positive_obstacle_grid_.info.height = height_;
        positive_obstacle_grid_.info.origin.position.x = -static_cast<double>(width_) * resolution_ / 2.0;
        positive_obstacle_grid_.info.origin.position.y = -static_cast<double>(height_) * resolution_ / 2.0;
        positive_obstacle_grid_.info.origin.position.z = 0.0;
        positive_obstacle_grid_.info.origin.orientation.w = 1.0;
        positive_obstacle_grid_.data.resize(width_ * height_, -1); 
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
        positive_obstacle_grid_.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        positive_obstacle_grid_.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        positive_obstacle_grid_.info.origin.position.z = odom_.pose.pose.position.z;
        RCLCPP_INFO(this->get_logger(), "Got Odometry and Updated Map position");
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        RCLCPP_INFO(this->get_logger(), "Segmented Cloud Size, %ld", cloud.size());

        // collect valid points
        pcl::PointCloud<pcl::PointXYZI>::Ptr valid_points(new pcl::PointCloud<pcl::PointXYZI>);
        for (const pcl::PointXYZI& point : cloud.points){
            double xy_distance = sqrt(pow(point.x - odom_.pose.pose.position.x, 2) + pow(point.y - odom_.pose.pose.position.y, 2));
            double z_distance = point.z - odom_.pose.pose.position.z;
            if (point.intensity == 99.0 && xy_distance > 0.75 && z_distance < 0.75){
                valid_points->push_back(point);
            }
        }

        // cluster valid points
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(valid_points);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(positive_obstacle_grid_.info.resolution);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(valid_points);
        ec.extract(cluster_indices);

        // Reset grid
        fill(positive_obstacle_grid_.data.begin(), positive_obstacle_grid_.data.end(), 0);

        // for each cluster, popoulate the occupancy_grid
        for (const pcl::PointIndices& indicies : cluster_indices){
            pcl::PointCloud<pcl::PointXYZI>::Ptr cluster_cloud(new pcl::PointCloud<pcl::PointXYZI>);
            pcl::PointCloud<pcl::PointXYZI>::Ptr hull(new pcl::PointCloud<pcl::PointXYZI>);
            for (int index: indicies.indices){
                pcl::PointXYZI point = valid_points->points[index];

                // project this point into the positive obstacle grid
                int grid_x = static_cast<int>((point.x - positive_obstacle_grid_.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point.y - positive_obstacle_grid_.info.origin.position.y) / resolution_);
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    positive_obstacle_grid_.data[grid_y * width_ + grid_x] = 100; 
                }

                cluster_cloud->push_back(point);
            }
        }

        positive_obstacle_grid_.header.stamp = this->now();
        positive_obstacle_grid_pub_->publish(positive_obstacle_grid_);
        RCLCPP_INFO(this->get_logger(), "Published Positive Occupancy Grid");

        nav_msgs::msg::OccupancyGrid dilated_positive_obstacle_grid = dialate_occupancy_grid();
        dilated_positive_obstacle_grid_pub_->publish(dilated_positive_obstacle_grid);
        RCLCPP_INFO(this->get_logger(), "Published Dialated Positive Occupancy Grid");
    }

    nav_msgs::msg::OccupancyGrid dialate_occupancy_grid(){
        // initialize relevant data
        nav_msgs::msg::OccupancyGrid dilated_positive_obstacle_grid;
        dilated_positive_obstacle_grid.header = positive_obstacle_grid_.header;
        dilated_positive_obstacle_grid.info = positive_obstacle_grid_.info;
        dilated_positive_obstacle_grid.data = positive_obstacle_grid_.data;

        cv::Mat grid_map(height_, width_, CV_8UC1, const_cast<int8_t*>(dilated_positive_obstacle_grid.data.data()));
        cv::Mat obstacle_mask = (grid_map == 100);

        int expansion_pixels = static_cast<int>(std::ceil(dialation_meters_ / resolution_));
        int kernel_size = 2 * expansion_pixels + 1;
        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernel_size, kernel_size));
        cv::Mat dilated_mask;
        cv::dilate(obstacle_mask, dilated_mask, kernel);

        for (int y = 0; y < height_; ++y) {
            for (int x = 0; x < width_; ++x) {
                if (dilated_mask.at<uint8_t>(y, x)) {
                    dilated_positive_obstacle_grid.data[y * width_ + x] = 100;
                }
            }
        }

        return dilated_positive_obstacle_grid;
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr positive_obstacle_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dilated_positive_obstacle_grid_pub_;

    nav_msgs::msg::Odometry odom_;
    nav_msgs::msg::OccupancyGrid positive_obstacle_grid_;
    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;
    double dialation_meters_ = 0.75;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::PositiveObstacleDetectionNode)