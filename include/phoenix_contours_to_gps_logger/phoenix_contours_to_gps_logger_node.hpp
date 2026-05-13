#pragma once

#include <geographic_msgs/msg/geo_path.hpp>  // optional custom message for a list of points
#include <geographic_msgs/msg/geo_point.hpp>
#include <opencv2/core/types.hpp>
#include <robot_localization/srv/to_ll.hpp>  // old ROS 2 style? It's usually "robot_localization/srv/to_ll.h" or check your package.
#include <string>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/msg/path.hpp"
#include "phnx_msgs/msg/contours.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class phoenix_contours_to_gps_logger : public rclcpp::Node {
private:
    // Camera info sub & model vars
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr rgb_info_sub;
    image_geometry::PinholeCameraModel rgb_model;

    // std::optional<nav_msgs::msg::Path>
    std::unique_ptr<std::optional<nav_msgs::msg::Path>> Backend;

    // TF2 stuff
    std::unique_ptr<tf2_ros::TransformListener> tf2_listener;
    std::unique_ptr<tf2_ros::Buffer> tf2_buffer;

    // navsat node translating stuff
    rclcpp::Client<ToLL>::SharedPtr to_ll_client_;

    // pubs for the contours as translated!
    rclcpp::Publisher<geographic_msgs::msg::GeoPath>::SharedPtr gps_left_pub_;
    rclcpp::Publisher<geographic_msgs::msg::GeoPath>::SharedPtr gps_right_pub_;

public:
    phoenix_contours_to_gps_logger(const rclcpp::NodeOptions& options);

    /// subscriber callback
    void contours_cb(phnx_msgs::msg::Contours::SharedPtr msg, image_geometry::PinholeCameraModel camera_rgb);
};
