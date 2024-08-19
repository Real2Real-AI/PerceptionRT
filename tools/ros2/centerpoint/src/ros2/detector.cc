#include "ros2/detector.hpp"

ROS2Detector::ROS2Detector() : Node("centerpoint")
{
  this->declare_parameter("config_path");
  this->declare_parameter("model_path");
  m_config = YAML::LoadFile(this->get_parameter("config_path").as_string());

  m_centerpoint = std::make_shared<CenterPoint>(m_config, this->get_parameter("model_path").as_string());

  std::string sub_topic_name = m_config["centerpoint"]["sub"].as<std::string>();
  std::string pub_topic_name = m_config["centerpoint"]["pub"].as<std::string>();
  m_ros_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(sub_topic_name, 1,
    std::bind(&ROS2Detector::callback, this, std::placeholders::_1));
  m_ros_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>(pub_topic_name, 1);
  m_pcl_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);

  size_t max_points = m_config["centerpoint"]["max_points"].as<size_t>();
  size_t max_points_feature = max_points * m_config["voxelization"]["num_feature"].as<size_t>();
  m_points.reserve(max_points_feature);
}

size_t ROS2Detector::getPoints(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pcl_cloud)
{
  m_points.clear();
  int num_points = pcl_cloud->size();
  for (int i = 0; i < num_points; i++) {
    m_points.push_back(pcl_cloud->points[i].x);
    m_points.push_back(pcl_cloud->points[i].y);
    m_points.push_back(pcl_cloud->points[i].z);
    m_points.push_back(pcl_cloud->points[i].intensity);
  }
  m_input_points = m_points.data();

  return num_points;
}

void ROS2Detector::callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  pcl::fromROSMsg(*msg, *m_pcl_cloud);
  std::shuffle(m_pcl_cloud->begin(), m_pcl_cloud->end(), m_rng);
  size_t num_points = getPoints(m_pcl_cloud);

  m_centerpoint->forward(num_points, m_input_points);
  m_boxes = m_centerpoint->getBoxesPointer();

  pubishBoxes();
}

void ROS2Detector::pubishBoxes()
{
  visualization_msgs::msg::MarkerArray msg;
  for (int i = 0; i < m_boxes->size(); i++) {
    Box& box = (*m_boxes)[i];
    if (box.score() < m_score_threshold) {
      continue;
    }
    visualization_msgs::msg::Marker marker;
    marker.id = i;
    marker.header.frame_id = "base_link";
    marker.header.stamp = this->get_clock()->now();
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;

    marker.pose.position.x = box.x();
    marker.pose.position.y = box.y();
    marker.pose.position.z = box.z();

    marker.scale.x = box.l();
    marker.scale.y = box.w();
    marker.scale.z = box.h();

    tf2::Quaternion q;
    q.setRPY(0, 0, box.yaw());
    marker.pose.orientation.w = q.w();
    marker.pose.orientation.x = q.x();
    marker.pose.orientation.y = q.y();
    marker.pose.orientation.z = q.z();

    int cls = box.cls();
    if (cls == 0) {
      marker.color.r = 1.0;
      marker.color.g = 0.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
    }
    else if (cls == 1){
      marker.color.r = 0.0;
      marker.color.g = 1.0;
      marker.color.b = 0.0;
      marker.color.a = 0.5;
    }
    else {
      marker.color.r = 0.0;
      marker.color.g = 0.0;
      marker.color.b = 1.0;
      marker.color.a = 0.5;
    }
    marker.lifetime = rclcpp::Duration(0, 300000000);
    msg.markers.push_back(marker);
  }
  m_ros_pub->publish(msg);
}