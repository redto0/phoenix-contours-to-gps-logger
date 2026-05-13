#include <cmath>
#include <fstream>
#include <iostream>
#include <optional>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string_view>
#include <vector>

#include "image_geometry/pinhole_camera_model.h"
#include "nav_msgs/msg/path.hpp"

namespace backend {
std::vector<cv::Point2d> cameraPixelToGroundPos(std::vector<cv::Point2d>& pixels,
                                                image_geometry::PinholeCameraModel& rgb_info_sub);
nav_msgs::msg::Path cameraPixelToGroundPath(std::vector<cv::Point2d>& pixels,
                                            const image_geometry::PinholeCameraModel& rgb_info_sub, float camera_height,
                                            std::string frame_id);
std::optional<nav_msgs::msg::Path> create_path(std::vector<cv::Point2d>& left_contours,
                                               std::vector<cv::Point2d>& right_contours,
                                               image_geometry::PinholeCameraModel& camera_info, std::string frame_id);
}  // namespace backend
