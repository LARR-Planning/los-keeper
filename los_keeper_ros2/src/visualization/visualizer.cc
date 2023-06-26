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
ObstaclePathVisualizationMsg
Visualizer::VisualizeObstaclePathArray(const PrimitiveList &obstacle_primitive) {
  visualization_msgs::msg::MarkerArray visual_output;
  bool is_primitive_generated = not obstacle_primitive.empty();
  if (is_primitive_generated) {
    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.color.a = 0.2;
    line_strip.color.r = 0.3;
    line_strip.color.g = 0.3;
    line_strip.color.b = 0.3;
    line_strip.scale.x = 0.01;
    line_strip.action = visualization_msgs::msg::Marker::MODIFY;
    line_strip.pose.orientation.w = 1.0;
    int num_time_sample = 20;
    std::vector<float> time_seq;
    float seg_t0 = obstacle_primitive[0].px.GetTimeInterval()[0];
    float seg_tf = obstacle_primitive[0].px.GetTimeInterval()[1];
    for (int i = 0; i < num_time_sample; i++)
      time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) / (float)(num_time_sample - 1));
    geometry_msgs::msg::Point temp_point;
    for (int i = 0; i < obstacle_primitive.size(); i++) { // the number of targets
      line_strip.points.clear();
      line_strip.id = i;
      line_strip.ns = std::to_string(i);
      for (int k = 0; k < num_time_sample; k++) {
        temp_point.x = obstacle_primitive[i].px.GetValue(time_seq[k]);
        temp_point.y = obstacle_primitive[i].py.GetValue(time_seq[k]);
        temp_point.z = obstacle_primitive[i].pz.GetValue(time_seq[k]);
        line_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strip);
    }
  }
  return visual_output;
}
TargetBestVisualizationMsg
Visualizer::VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexList &best_indices) {
  TargetBestVisualizationMsg visual_output;
  bool is_prediction_generated = not best_indices.empty();
  //  printf("best_indices size: %d.\n", (int)best_indices.size());
  if (is_prediction_generated) {
    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.color.a = 1.0;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.scale.x = 0.05;
    line_strip.action = visualization_msgs::msg::Marker::MODIFY;
    line_strip.pose.orientation.w = 1.0;
    int num_time_sample = 20;
    vector<float> time_seq;
    float seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
    float seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
    for (int i = 0; i < num_time_sample; i++)
      time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) / (float)(num_time_sample - 1));
    geometry_msgs::msg::Point temp_point;
    for (int i = 0; i < best_indices.size(); i++) {
      line_strip.points.clear();
      line_strip.id = i;
      line_strip.ns = std::to_string(i);
      for (int j = 0; j < num_time_sample; j++) {
        temp_point.x = primitive_list[i][best_indices[i]].px.GetValue(time_seq[j]);
        temp_point.y = primitive_list[i][best_indices[i]].py.GetValue(time_seq[j]);
        temp_point.z = primitive_list[i][best_indices[i]].pz.GetValue(time_seq[j]);
        line_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strip);
    }
  }
  return TargetBestVisualizationMsg();
}