#include <los_keeper_ros2/visualization/visualizer.h>

using namespace los_keeper;

Visualizer::Visualizer() {}
Visualizer::Visualizer(const VisualizationParameters &parameters) : parameters_(parameters) {}
void Visualizer::UpdateTime(const rclcpp::Time &time) { time_ = time; }

visualization_msgs::msg::Marker Visualizer::DeriveSomeDebugInfo(const int some_debug_info) const {
  visualization_msgs::msg::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = time_;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;
  marker.pose.position.x = some_debug_info;
  marker.scale.x = parameters_.marker_size;
  marker.color.a = 1.0;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  return marker;
}