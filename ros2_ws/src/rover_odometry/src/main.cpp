#include <exception>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "rover_odometry/odometry_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    rclcpp::spin(std::make_shared<rover_odometry::OdometryNode>());
  } catch (const std::exception & ex) {
    RCLCPP_FATAL(
      rclcpp::get_logger("rover_odometry"),
      "Odometry node failed: %s",
      ex.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}