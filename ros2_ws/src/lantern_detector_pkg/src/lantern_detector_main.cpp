#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "lantern_detector_pkg/lantern_detector_node.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<::lantern_detector_pkg::LanternDetectorNode>(rclcpp::NodeOptions());
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
