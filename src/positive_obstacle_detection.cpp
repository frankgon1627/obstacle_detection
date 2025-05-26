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
        averaged_positive_obstacle_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/averaged_positive_obstacle_grid", 10);
        dilated_positive_obstacle_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/dilated_positive_obstacle_grid", 10);
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odom_ = *msg;
    }

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        pcl::PointCloud<pcl::PointXYZI> cloud;
        pcl::fromROSMsg(*msg, cloud);

        // collect valid points
        pcl::PointCloud<pcl::PointXYZI>::Ptr valid_points = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
        for (const pcl::PointXYZI& point : cloud.points){
            double xy_distance = sqrt(pow(point.x - odom_.pose.pose.position.x, 2) + pow(point.y - odom_.pose.pose.position.y, 2));
            double z_distance = point.z - odom_.pose.pose.position.z;
            if (point.intensity == 99.0 && xy_distance > 0.75 && z_distance < 0.75){
                valid_points->push_back(point);
            }
        }

        // cluster valid points
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        tree->setInputCloud(valid_points);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(resolution_);
        ec.setMinClusterSize(5);
        ec.setMaxClusterSize(10000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(valid_points);
        ec.extract(cluster_indices);
        
        // initialize the occupancy grid
        nav_msgs::msg::OccupancyGrid positive_obstacle_grid = nav_msgs::msg::OccupancyGrid();
        positive_obstacle_grid.header.stamp = this->now();
        positive_obstacle_grid.header.frame_id = "odom";
        positive_obstacle_grid.info.resolution = resolution_;
        positive_obstacle_grid.info.width = width_;
        positive_obstacle_grid.info.height = height_;
        positive_obstacle_grid.info.origin.position.x = odom_.pose.pose.position.x - static_cast<double>(width_) * resolution_ / 2.0;
        positive_obstacle_grid.info.origin.position.y = odom_.pose.pose.position.y - static_cast<double>(height_) * resolution_ / 2.0;
        positive_obstacle_grid.info.origin.position.z = odom_.pose.pose.position.z;
        positive_obstacle_grid.info.origin.orientation.w = 1.0;
        positive_obstacle_grid.data.resize(width_ * height_, -1); 
        fill(positive_obstacle_grid.data.begin(), positive_obstacle_grid.data.end(), 0);

        // for each cluster, popoulate the occupancy_grid
        for (const pcl::PointIndices& indicies : cluster_indices){
            for (int index: indicies.indices){
                pcl::PointXYZI point = valid_points->points[index];

                // project this point into the positive obstacle grid
                int grid_x = static_cast<int>((point.x - positive_obstacle_grid.info.origin.position.x) / resolution_);
                int grid_y = static_cast<int>((point.y - positive_obstacle_grid.info.origin.position.y) / resolution_);
                if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_) {
                    positive_obstacle_grid.data[grid_y * width_ + grid_x] = 100; 
                }
            }
        }

        // store the latest positive obstacle grid and remove the oldest if over max size
        if (positive_obstacle_grid_deque_.size() >= deque_max_size_){
            positive_obstacle_grid_deque_.pop_front();
        }
        positive_obstacle_grid_deque_.push_back(positive_obstacle_grid);
        if (positive_obstacle_grid_pub_->get_subscription_count() > 0){
            positive_obstacle_grid_pub_->publish(positive_obstacle_grid);
        }

        nav_msgs::msg::OccupancyGrid averaged_positive_obstacle_grid = make_averaged_positive_obstacle_grid();
        if (averaged_positive_obstacle_grid_pub_->get_subscription_count() > 0){
            averaged_positive_obstacle_grid_pub_->publish(averaged_positive_obstacle_grid);
        }

        nav_msgs::msg::OccupancyGrid dilated_positive_obstacle_grid = dialate_occupancy_grid(averaged_positive_obstacle_grid);
        if (dilated_positive_obstacle_grid_pub_->get_subscription_count() > 0){
            dilated_positive_obstacle_grid_pub_->publish(dilated_positive_obstacle_grid);
        }

        RCLCPP_INFO(this->get_logger(), "Published Positive and Dialated Occupancy Grid");
    }

    nav_msgs::msg::OccupancyGrid make_averaged_positive_obstacle_grid(){
        nav_msgs::msg::OccupancyGrid averaged_positive_obstacle_grid;
        averaged_positive_obstacle_grid.header = positive_obstacle_grid_deque_.back().header;
        averaged_positive_obstacle_grid.info = positive_obstacle_grid_deque_.back().info;
        averaged_positive_obstacle_grid.data = vector<int8_t>(width_ * height_, 0);
        double averaged_map_origin_x = averaged_positive_obstacle_grid.info.origin.position.x;
        double averaged_map_origin_y = averaged_positive_obstacle_grid.info.origin.position.y;

        for(size_t cell_index=0; cell_index < averaged_positive_obstacle_grid.data.size(); cell_index++){
            // get the 2D position of the current cell
            int averaged_map_cell_i = cell_index % width_;  
            int averaged_map_cell_j = cell_index / height_;   
            double averaged_map_cell_x = averaged_map_origin_x + (averaged_map_cell_i + 0.5) * resolution_;
            double averaged_map_cell_y = averaged_map_origin_y + (averaged_map_cell_j + 0.5) * resolution_;

            // extract the cell_value of all previous maps at the given 2D location
            int non_zero_values = 0;
            int maps_included = 0;
            for(nav_msgs::msg::OccupancyGrid& old_map : positive_obstacle_grid_deque_){
                geometry_msgs::msg::Pose old_map_origin = old_map.info.origin;
                int old_map_cell_j = int((averaged_map_cell_x - old_map_origin.position.x) / resolution_);
                int old_map_cell_i = int((averaged_map_cell_y - old_map_origin.position.y) / resolution_);

                // check if we are within bounds
                if(0 <= old_map_cell_i && old_map_cell_i < height_ && 0 <= old_map_cell_j && old_map_cell_j < width_){
                    // check if the cell value is non-zero
                    if (old_map.data[old_map_cell_i * width_ + old_map_cell_j] == 100) {
                        ++non_zero_values;
                    }
                }
                ++maps_included;
            }
            
            // set cell value if the cell_value_accumulation is greater than 80%
            if (non_zero_values > 0.75 * maps_included) {
                averaged_positive_obstacle_grid.data[cell_index] = 100;
            }
        }
        return averaged_positive_obstacle_grid;
    }

    nav_msgs::msg::OccupancyGrid dialate_occupancy_grid(nav_msgs::msg::OccupancyGrid& averaged_positive_obstacle_grid) {
        // initialize relevant data
        nav_msgs::msg::OccupancyGrid dilated_positive_obstacle_grid;
        dilated_positive_obstacle_grid.header = averaged_positive_obstacle_grid.header;
        dilated_positive_obstacle_grid.info = averaged_positive_obstacle_grid.info;
        dilated_positive_obstacle_grid.data = averaged_positive_obstacle_grid.data;

        cv::Mat grid_map(height_, width_, CV_8UC1, const_cast<int8_t*>(dilated_positive_obstacle_grid.data.data()));
        cv::Mat obstacle_mask = (grid_map == 100);

        int expansion_pixels = static_cast<int>(std::ceil(dialation_meters_ / resolution_));
        int kernel_size = expansion_pixels + 1;
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
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr averaged_positive_obstacle_grid_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr dilated_positive_obstacle_grid_pub_;

    deque<nav_msgs::msg::OccupancyGrid> positive_obstacle_grid_deque_;
    long unsigned int deque_max_size_ = 30;

    nav_msgs::msg::Odometry odom_;

    double resolution_ = 0.2;
    int width_ = 100;
    int height_ = 100;
    double dialation_meters_ = 0.75;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::PositiveObstacleDetectionNode)
