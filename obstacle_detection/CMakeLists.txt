cmake_minimum_required(VERSION 3.8)
project(obstacle_detection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(obstacle_detection_msgs)
find_package(custom_msgs_pkg REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(obstacle_detection_container SHARED src/positive_obstacle_detection.cpp src/negative_obstacle_detection.cpp src/combine_maps.cpp)
ament_target_dependencies(obstacle_detection_container 
    rclcpp 
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    visualization_msgs
    obstacle_detection_msgs
    custom_msgs_pkg
    pcl_conversions
    pcl_ros
    tf2_ros
    tf2_geometry_msgs
    OpenCV
)

rclcpp_components_register_node(
    obstacle_detection_container
    PLUGIN "obstacle_detection::PositiveObstacleDetectionNode"
    EXECUTABLE positive_obstacle_detection_node
)

rclcpp_components_register_node(
    obstacle_detection_container
    PLUGIN "obstacle_detection::NegativeObstacleDetectionNode"
    EXECUTABLE negative_obstacle_detection_node
)

rclcpp_components_register_node(
    obstacle_detection_container
    PLUGIN "obstacle_detection::CombinerNode"
    EXECUTABLE combiner_node
)

ament_export_dependencies(
    rclcpp 
    std_msgs 
    sensor_msgs 
    nav_msgs 
    geometry_msgs 
    visualization_msgs
    obstacle_detection_msgs
    custom_msgs_pkg
    pcl_conversions 
    pcl_ros 
    tf2_ros 
    tf2_geometry_msgs
    OpenCV)
ament_package()

install(
    TARGETS obstacle_detection_container
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME} 
)

