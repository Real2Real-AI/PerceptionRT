# ifndef ROS2_DETECTOR_HPP
# define ROS2_DETECTOR_HPP

# include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/msg/quaternion.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

# include <memory>

# include "centerpoint/centerpoint.hpp"

class ROS2Detector : public rclcpp::Node
{
public:
  ROS2Detector();

  ~ROS2Detector() = default;

  void callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

private:
  size_t getPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud);

  void pubishBoxes();


private:
  std::shared_ptr<CenterPoint> m_centerpoint;
  YAML::Node m_config;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_ros_sub;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr m_ros_pub;

  pcl::PointCloud<pcl::PointXYZI>::Ptr m_pcl_cloud;
  std::minstd_rand0 m_rng = std::default_random_engine{};

  float* m_input_points = nullptr;
  std::vector<float> m_points;
  std::vector<Box>* m_boxes;
  float m_score_threshold;
};

# endif // ROS2_DETECTOR_HPP
