# include <iostream>
// #include "centerpoint/centerpoint.hpp"
# include "ros2/detector.hpp"

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // auto centerpoint_node = std::make_shared<CenterPoint>();
  auto detector = std::make_shared<ROS2Detector>();
  rclcpp::spin(detector);
  rclcpp::shutdown();

  return 0;
}