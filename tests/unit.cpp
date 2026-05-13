#include <gtest/gtest.h>

#include <rclcpp/rclcpp.hpp>

#include "phoenix_contours_to_gps_logger/phoenix_contours_to_gps_logger_node.hpp"

TEST(phoenix_contours_to_gps_logger, Test1) {}

int main(int argc, char** argv) {
    rclcpp::init(0, nullptr);

    ::testing::InitGoogleTest(&argc, argv);
    auto res = RUN_ALL_TESTS();

    rclcpp::shutdown();
    return res;
}