#include "node.hpp"

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp_lifecycle::LifecycleNode::SharedPtr node = std::make_shared<ArucoControllerNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();

  return 0;
}