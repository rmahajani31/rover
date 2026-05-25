#include "gamepad_adapter/adapter_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gamepad_adapter::AdapterNode>());
  rclcpp::shutdown();
  return 0;
}
