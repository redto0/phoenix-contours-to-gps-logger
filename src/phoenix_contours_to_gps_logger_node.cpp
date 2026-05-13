#include "phoenix_contours_to_gps_logger/phoenix_contours_to_gps_logger_node.hpp"

// For _1
using namespace std::placeholders;

phoenix_contours_to_gps_logger::phoenix_contours_to_gps_logger(const rclcpp::NodeOptions& options)
    : Node("phoenix_contours_to_gps_logger", options) {
    // RGB_INFO PARAMETER DO NOT DELETE
    this->declare_parameter("camera_frame", "mid_cam_link");

    // Frame params
    this->declare_parameter("path_frame", "odom");
    this->declare_parameter("contour_frame", "map");

    this->rgb_info_sub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        "/camera/mid/rgb/camera_info", 1,
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr ci) { this->rgb_model.fromCameraInfo(ci); });

    this->poly_sub = this->create_subscription<phnx_msgs::msg::Contours>(
        "/road/Contours", 1, [this](phnx_msgs::msg::Contours::SharedPtr msg) {
            this->contours_cb(msg, this->rgb_model);
        });  ///  TODO fix paras

    RCLCPP_INFO(this->get_logger(), "PolynomialPlannerAi Node Started! Waiting for contour data...");
    // TF2 things
    this->tf2_buffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    this->tf2_listener = std::make_unique<tf2_ros::TransformListener>(*this->tf2_buffer);

    to_ll_client_ = this->create_client<ToLL>("toLL");
    if (!to_ll_client_->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(this->get_logger(), "toLL service not available – node will not convert to GPS");
    }

    // Publishers for GPS paths
    gps_left_pub_ = this->create_publisher<geographic_msgs::msg::GeoPath>("~/gps/left_boundary", 10);
    gps_right_pub_ = this->create_publisher<geographic_msgs::msg::GeoPath>("~/gps/right_boundary", 10);

    RCLCPP_INFO(this->get_logger(), "Node started, waiting for contour data...");
}

void phoenix_contours_to_gps_logger::contours_cb(phnx_msgs::msg::Contours::SharedPtr msg,
                                                 image_geometry::PinholeCameraModel camera_rgb) {
    // fix msg->empty
    if (msg->left_contour.empty() && msg->right_contour.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty polynomial (non-AI)");
        return;
    }
    std::vector<geometry_msgs::msg::Vector3> left = msg->left_contour;
    std::vector<geometry_msgs::msg::Vector3> right = msg->right_contour;

    std::vector<cv::Point2d> cv_points_left;
    std::vector<cv::Point2d> cv_points_right;

    for (const auto& vec : left) {
        cv_points_left.emplace_back(vec.x, vec.y);  // Efficient in-place construction
    }

    for (const auto& vec : right) {
        cv_points_right.emplace_back(vec.x, vec.y);  // Efficient in-place construction
    }

    std::string p = "left contour size " + std::to_string(left.size());
    RCLCPP_INFO(this->get_logger(), p.c_str());

    p = "right contour size " + std::to_string(right.size());
    RCLCPP_INFO(this->get_logger(), p.c_str());

    //std::string frame_id = this->get_parameter("camera_frame").as_string();
    //std::string frame_id = "notemptystring";
    // TODO camera frame_id is wrong
    auto frame_id = this->get_parameter(std::string("camera_frame")).as_string();
    std::optional<nav_msgs::msg::Path> left_ground_contours_optional =
        backend::create_path(cv_points_left, camera_rgb, frame_id);
    nav_msgs::msg::Path left_ground_contours;

    std::optional<nav_msgs::msg::Path> right_ground_contours_optional =
        backend::create_path(cv_points_right, camera_rgb, frame_id);
    nav_msgs::msg::Path right_ground_contours;

    if (path_optional.has_value()) {
        path = path_optional.value();
        std::string p = std::to_string(path.poses.size());
        RCLCPP_INFO(this->get_logger(), p.c_str());
        path.header.frame_id = this->get_parameter(std::string("camera_frame")).as_string();
        path.header.stamp = this->get_clock()->now();

        try {
            // After you have the path in the 'map' frame:
            // path is nav_msgs::msg::Path with header.frame_id = "map"

            if (to_ll_client_->service_is_ready()) {
                // You can publish both left and right boundaries as GPS paths
                geographic_msgs::msg::GeoPath gps_path;
                gps_path.header.stamp = path.header.stamp;
                gps_path.header.frame_id = "gps";  // or "geodetic"

                for (const auto& pose : path.poses) {
                    auto request = std::make_shared<ToLL::Request>();
                    request->map_point = pose.pose.position;  // x,y,z in map frame

                    // Use async client to avoid blocking the callback
                    auto future = to_ll_client_->async_send_request(
                        request, [&gps_path, this, pub](rclcpp::Client<ToLL>::SharedFuture result) {
                            auto response = result.get();
                            if (response) {
                                gps_path.points.push_back(response->ll_point);
                            } else {
                                RCLCPP_WARN(get_logger(), "toLL call failed");
                            }
                        });

                    // Spin until all futures complete? Simpler: use synchronous call if few points.
                    // For demonstration, I'll show a synchronous version that is easier to integrate:
                }
            }
        } catch (tf2::LookupException& e) {
            RCLCPP_INFO(this->get_logger(), "Could not look up odom!");
            return;
        }

        this->gps_path_pub_->publish(gps_path);  // error invalid operator *path
                                                 // Extract and print coefficients
        // RCLCPP_INFO(this->get_logger(), "Received Polynomial Coefficients:");
    } else {
        std::string p = " error no path ";
        RCLCPP_INFO(this->get_logger(), p.c_str());
    }
    return;

    // this->pub->publish(*msg);
}
