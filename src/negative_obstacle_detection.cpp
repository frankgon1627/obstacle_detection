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

class NegativeObstacleDetectionNode : public rclcpp::Node
{
public:
    NegativeObstacleDetectionNode(const rclcpp::NodeOptions & options) : 
    Node("negative_obstacle_detection", options), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_){
        auto point_cloud_qos = rclcpp::QoS(10);
        point_cloud_qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        point_cloud_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/ouster/points", point_cloud_qos, bind(&NegativeObstacleDetectionNode::point_cloud_callback, this, placeholders::_1));
        occupancy_grid_subscriber_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "/obstacle_detection/dialated_positive_obstacle_grid", 10, bind(&NegativeObstacleDetectionNode::occupancy_grid_callback, this, placeholders::_1));

        risk_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/planners/risk_map", 10);
        filtered_risk_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/planners/filtered_risk_map", 10);
        combined_map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/planners/combined_map", 10);
        feature_points_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/planners/feature_points", 10);

        timer_ = this->create_wall_timer(chrono::milliseconds(100), bind(&NegativeObstacleDetectionNode::transform_callback, this));

        // initialize relevant LIDAR geometry arrays
        vertical_angles_ = {16.349001, 15.750999, 15.181001, 14.645001, 14.171, 13.595, 13.043,
                            12.509, 12.041, 11.471, 10.932, 10.401, 9.935001400000001, 9.3830004,
                            8.8369999, 8.3079996, 7.8410001, 7.2950006, 6.7539997, 6.2189999, 5.756,
                            5.2119999, 4.671, 4.1409998, 3.678, 3.1359999, 2.5969999, 2.0630002,
                            1.5980002, 1.061, 0.51800007, -0.011, -0.48699999, -1.0250001, -1.562,
                            -2.102, -2.566, -3.105, -3.648, -4.1849999, -4.652, -5.1820002, -5.7289996,
                            -6.2719998, -6.7420001, -7.2729998, -7.8119998, -8.362000500000001,
                            -8.833000200000001, -9.3629999, -9.906999600000001, -10.46, -10.938, -11.473,
                            -12.015, -12.581, -13.071, -13.604, -14.157001, -14.726998, -15.241, -15.773,
                            -16.337999, -16.917999};
        reverse(vertical_angles_.begin(), vertical_angles_.end());
        // store as radians
        for (size_t i=0; i < vertical_channels_; i++){
            vertical_angles_rad_[i] = vertical_angles_[i] * M_PI / 180.0;
        }
        // calcualte ground distance from LIDAR to point
        for (size_t i=0; i < vertical_channels_; i++){
            rho_i_[i] = lidar_height_ / tanf(-vertical_angles_rad_[i]);
        }
        // calculate the detlas between points
        for (size_t i=0; i < vertical_channels_; i++){
            size_t prev_index = (i==0) ? vertical_channels_ - 1 : i - 1;
            vdl_[i] = abs(rho_i_[i] - rho_i_[prev_index]);
        }

        RCLCPP_INFO(this->get_logger(), "Negative Obstacle Detection Node has been started");
    }

private:
    void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg){
        occupancy_grid_ = msg;
        map_height_ = occupancy_grid_->info.height;
        map_width_ = occupancy_grid_->info.width;
        map_resolution_ = occupancy_grid_->info.resolution;
    }

    void transform_callback(){
        if (!occupancy_grid_)
        {
            RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
            return;
        }

        try{
            string from_frame = "os_lidar";
            string to_frame = occupancy_grid_->header.frame_id;

            lidar_to_costmap_transform_ = tf_buffer_.lookupTransform(to_frame, from_frame, tf2::TimePointZero);
        }
        catch (const tf2::TransformException &ex){
            RCLCPP_ERROR(this->get_logger(), "Failed to get transform: %s", ex.what());
        }
    }

    void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (!occupancy_grid_)
        {
            RCLCPP_WARN(this->get_logger(), "Occupancy grid not received yet");
            return;
        }

        if (lidar_to_costmap_transform_.header.frame_id.empty())
        {
            RCLCPP_WARN(this->get_logger(), "Transform not received yet");
            return;
        }
        
        // convert the point cloud to a 3D array
        PointArray point_cloud_array = get_point_cloud_array(msg);

        // parallelize processing each scan column and extracting feature points
        FeaturePoints feature_points;
        array<future<FeaturePoints>, horizontal_channels_> futures;
        for (size_t col=0; col < horizontal_channels_; col++){
            futures[col] = async(launch::async, [this, &point_cloud_array, col]() { 
                return process_scan_column(point_cloud_array, col); 
            });
        }

        // collect the results from each future
        for (size_t col=0; col < horizontal_channels_; col++){
            FeaturePoints feature_points_col = futures[col].get();
            feature_points.insert(feature_points.end(), feature_points_col.begin(), feature_points_col.end());
        }
        publish_feature_points(feature_points);

        // extract relevant Occupancy Grid Information
        nav_msgs::msg::MapMetaData map_info = occupancy_grid_->info;
        geometry_msgs::msg::Pose map_origin = map_info.origin;

        // make a risk map and a combined map for publishing
        nav_msgs::msg::OccupancyGrid risk_map;
        risk_map.header = occupancy_grid_->header;
        risk_map.info = map_info;
        risk_map.data = vector<int8_t>(map_width_ * map_height_, 0);

        nav_msgs::msg::OccupancyGrid combined_map;
        combined_map.header = occupancy_grid_->header;
        combined_map.info = map_info;
        combined_map.data = occupancy_grid_->data;

        // get the transformation between the LIDAR frame and the costmap frame
        geometry_msgs::msg::Quaternion quaternion = lidar_to_costmap_transform_.transform.rotation;
        geometry_msgs::msg::Vector3 translation = lidar_to_costmap_transform_.transform.translation;
        tf2::Quaternion tf_quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
        tf2::Vector3 tf_translation = {translation.x, translation.y, translation.z};

        // use point correspondences to set the values in the occupancy grid
        for(array<float, 7> correspondence: feature_points){
            tf2::Vector3 point1 = {correspondence[0], correspondence[1], correspondence[2]};
            tf2::Vector3 point2 = {correspondence[3], correspondence[4], correspondence[5]};
            float risk = correspondence[6];

            // Filter out nonsensical feature points
            if (abs(point1.y()) < 1.0 && -4 < point1.x() && point1.x() < 0){
                continue;
            }

            // transform the points to the costmap frame
            tf2::Vector3 point1_cost_frame = tf2::quatRotate(tf_quaternion, point1) + tf_translation;
            tf2::Vector3 point2_cost_frame = tf2::quatRotate(tf_quaternion, point2) + tf_translation;
            float norm = sqrt(pow(point1_cost_frame.x(), 2) + pow(point1_cost_frame.y(), 2));

            // determine the 2D index in the cost_map data array
            int j1 = int((point1_cost_frame.x() - map_origin.position.x) / map_resolution_);
            int i1 = int((point1_cost_frame.y() - map_origin.position.y) / map_resolution_);
            int j2 = int((point2_cost_frame.x() - map_origin.position.x) / map_resolution_);
            int i2 = int((point2_cost_frame.y() - map_origin.position.y) / map_resolution_);

            vector<pair<int, int>> grid_cells = bresenham_line(i1, j1, i2, j2);
            for (pair<int, int> grid_cell : grid_cells){
                int cell_i = grid_cell.first;
                int cell_j = grid_cell.second;

                // ensure the cell is within the grid
                if(0 <= cell_i && cell_i < map_height_ && 0 <= cell_j && cell_j < map_width_){
                    int8_t cell_value = static_cast<int8_t>(min(norm*risk, 100.0f));
                    // set the risk value in the risk map
                    risk_map.data[cell_i * map_width_ + cell_j] = cell_value;
                }
            }
        }
        // store the latest risk map and remove the oldest if over max size
        if (risk_map_deque_.size() >= deque_max_size_){
            risk_map_deque_.pop_front();
        }
        risk_map_deque_.push_back(risk_map);

        // temporally filter over the last N risk maps
        nav_msgs::msg::OccupancyGrid filtered_risk_map = make_filtered_risk_map();
        risk_map_pub_->publish(risk_map);
        filtered_risk_map_pub_->publish(filtered_risk_map);

        // make combined risk map
        for(size_t i=0; i < filtered_risk_map.data.size(); i++){
            if (combined_map.data[i] == 0){
                combined_map.data[i] = filtered_risk_map.data[i];
            }
        }
        combined_map_pub_->publish(combined_map);
    }

    PointArray get_point_cloud_array(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        PointArray point_cloud_array;
        
        if (msg->height != vertical_channels_ || msg->width != horizontal_channels_)
        {
            RCLCPP_ERROR(this->get_logger(), "Point cloud dimensions do not match expected values");
            return point_cloud_array;
        }

        sensor_msgs::PointCloud2ConstIterator<float> it_x(*msg, "x");
        sensor_msgs::PointCloud2ConstIterator<float> it_y(*msg, "y");
        sensor_msgs::PointCloud2ConstIterator<float> it_z(*msg, "z");

        for (size_t i = 0; i < msg->height; i++){
            for (size_t j = 0; j < msg->width; j++, ++it_x, ++it_y, ++it_z){
                // if the norm is under a threshold, set values to nan
                double norm = sqrt((*it_x) * (*it_x) + (*it_y) * (*it_y) + (*it_z) * (*it_z));
                if (norm < 0.5){
                    point_cloud_array[i][j] = {numeric_limits<float>::quiet_NaN(), 
                                                numeric_limits<float>::quiet_NaN(), 
                                                numeric_limits<float>::quiet_NaN()};
                }
                else{
                    point_cloud_array[i][j] = {*it_x, *it_y, *it_z};
                }
            }
        }
        return point_cloud_array;
    }

    FeaturePoints process_scan_column(PointArray& point_cloud_array, size_t col){
        FeaturePoints feature_points_col;
        array<float, 3>* last_point = nullptr;
        
        for (int row=vertical_channels_ - 1; row >= 0; row--){
            array<float, 3>& current_point = point_cloud_array[row][col];

            // skip over any current_points that are too closer to the LIDAR
            if (isnan(current_point[0]) || isnan(current_point[1]) || isnan(current_point[2])){
                continue;
            }

            // initialize first valid current_point
            if(last_point == nullptr){
                last_point = &current_point;
                continue;
            }

            // clear positive obstacle
            if (current_point[2] > 0.5){
                break;
            }

            // if the current_point is outside the costmap, disregard rest of data on this line
            //if (sqrt(pow(current_point[0], 2) + pow(current_point[1], 2)) > occupancy_grid_->info.width * sqrt(2)){
            //    break;
            //}

            double expected_deviation = 2.0 * vdl_[vertical_channels_ - row - 1];
            float deviation = sqrt(pow(current_point[0] - (*last_point)[0], 2) + pow(current_point[1] - (*last_point)[1], 2));
            // if the current point is within the threshold distance, update the last valid point
            if (deviation < expected_deviation){
                last_point = &current_point;
            }
            // exceeded threhsold, mark this pair as a pair of feature points and store deviation
            else{
                float risk  = deviation / expected_deviation;
                feature_points_col.push_back({(*last_point)[0], (*last_point)[1], (*last_point)[2], 
                                                current_point[0], current_point[1], current_point[2], risk});
                last_point = &current_point;
            }
        }
        return feature_points_col;
    }

    nav_msgs::msg::OccupancyGrid make_filtered_risk_map(){
        nav_msgs::msg::OccupancyGrid filtered_risk_map;
        filtered_risk_map.header = occupancy_grid_->header;
        filtered_risk_map.info = occupancy_grid_->info;
        filtered_risk_map.data = vector<int8_t>(occupancy_grid_->info.width * occupancy_grid_->info.height, 0);
        double filtered_map_origin_x = filtered_risk_map.info.origin.position.x;
        double filtered_map_origin_y = filtered_risk_map.info.origin.position.y;

        for(size_t cell_index=0; cell_index < filtered_risk_map.data.size(); cell_index++){
            // get the 2D position of the current cell
            int filtered_map_cell_i = cell_index % map_width_;  
            int filtered_map_cell_j = cell_index / map_height_;   
            double filtered_map_cell_x = filtered_map_origin_x + (filtered_map_cell_i + 0.5) * map_resolution_;
            double filtered_map_cell_y = filtered_map_origin_y + (filtered_map_cell_j + 0.5) * map_resolution_;

            // extract the cell_value of all previous maps at the given 2D location
            int cell_value_accumulation = 0;
            int maps_included = 0;
            for(nav_msgs::msg::OccupancyGrid& old_map : risk_map_deque_){
                geometry_msgs::msg::Pose old_map_origin = old_map.info.origin;
                int old_map_cell_j = int((filtered_map_cell_x - old_map_origin.position.x) / map_resolution_);
                int old_map_cell_i = int((filtered_map_cell_y - old_map_origin.position.y) / map_resolution_);

                // add to the accumulated cell value if within bounds
                if(0 <= old_map_cell_i && old_map_cell_i < map_height_ && 0 <= old_map_cell_j && old_map_cell_j < map_width_){
                    cell_value_accumulation += old_map.data[old_map_cell_i * map_width_ + old_map_cell_j];
                    ++maps_included;
                }
            }

            // save filtered cell value into filtered risk map
            filtered_risk_map.data[cell_index] = static_cast<uint8_t>(cell_value_accumulation / maps_included);
        }  
        return filtered_risk_map;
    }

    vector<pair<int, int>> bresenham_line(int i1, int j1, int i2, int j2){
        vector<pair<int, int>> points;
        int dx = abs(i2 - i1);
        int dy = abs(j2 - j1);
        int sx = (i1 < i2) ? 1 : -1;
        int sy = (j1 < j2) ? 1 : -1;
        int err = dx - dy;

        while (true){
            points.push_back(make_pair(i1, j1));
            if (i1 == i2 && j1 == j2) break;
            int err2 = err * 2;
            if (err2 > -dy){
                err -= dy;
                i1 += sx;
            }
            if (err2 < dx){
                err += dx;
                j1 += sy;
            }
        }
        return points;
    }

    void publish_feature_points(FeaturePoints& feature_points){
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "os_lidar";
        marker.header.stamp = this->now();
        marker.ns = "feature_points";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;

        for (const array<float, 7>& point : feature_points){
            geometry_msgs::msg::Point p1, p2;
            p1.x = point[0];
            p1.y = point[1];
            p1.z = point[2];
            p2.x = point[3];
            p2.y = point[4];
            p2.z = point[5];
            marker.points.push_back(p1);
            marker.points.push_back(p2);

            std_msgs::msg::ColorRGBA color;
            color.a = 100.0;
            color.r = 0;
            color.g = 0;
            color.b = 255.0;
            marker.colors.push_back(color);
            marker.colors.push_back(color);
        }

        feature_points_pub_->publish(marker);
    }

    // void publish_scan_line()

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_subscriber_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr risk_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr filtered_risk_map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr combined_map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr feature_points_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Obstacle map variables
    nav_msgs::msg::OccupancyGrid::SharedPtr occupancy_grid_;
    int map_height_;
    int map_width_;
    double map_resolution_;
    geometry_msgs::msg::TransformStamped lidar_to_costmap_transform_;
    deque<nav_msgs::msg::OccupancyGrid> risk_map_deque_;
    long unsigned int deque_max_size_ = 30;

    // LIDAR geometry variables
    const double lidar_height_ = 0.4082;
    array<double, 64> vertical_angles_;
    array<double, 64> vertical_angles_rad_;
    array<double, 64> rho_i_;
    array<double, 64> vdl_;
};
}

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(obstacle_detection::NegativeObstacleDetectionNode)