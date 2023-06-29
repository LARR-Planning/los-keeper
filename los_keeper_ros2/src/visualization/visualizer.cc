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
    line_strip.ns = "obstacle_path";
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
      for (int k = 0; k < num_time_sample; k++) {
        temp_point.x = obstacle_primitive[i].px.GetValue(time_seq[k]);
        temp_point.y = obstacle_primitive[i].py.GetValue(time_seq[k]);
        temp_point.z = obstacle_primitive[i].pz.GetValue(time_seq[k]);
        line_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strip);
    }
    //    visualization_msgs::msg::Marker erase_marker;
    //    erase_marker.type = visualization_msgs::msg::Marker::DELETEALL;
    //    erase_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    //    erase_marker.ns = line_strip.ns;
    //    erase_marker.id = (int)obstacle_primitive.size();
    //    erase_marker.header.frame_id = "map";
    //    visual_output.markers.push_back(erase_marker);
  }
  return visual_output;
}
TargetBestPathVisualizationMsg
Visualizer::VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexList &best_indices) {
  TargetBestPathVisualizationMsg visual_output;
  bool is_best_indices_generated = not best_indices.empty();
  bool is_primitive_generated_big = not primitive_list.empty();
  bool is_primitive_generated = true;
  if (is_primitive_generated_big) {
    for (int i = 0; i < primitive_list.size(); i++) {
      if (primitive_list[i].empty())
        is_primitive_generated = false;
    }
  }
  is_primitive_generated = is_primitive_generated and is_primitive_generated_big;

  if (is_best_indices_generated and is_primitive_generated) {
    //      printf("best_indices size: %d.\n", (int)best_indices.size());
    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.color.a = 0.5;
    line_strip.color.r = 1.0;
    line_strip.color.g = 0.0;
    line_strip.color.b = 0.0;
    line_strip.scale.x = 0.03;
    line_strip.action = visualization_msgs::msg::Marker::MODIFY;
    line_strip.pose.orientation.w = 1.0;

    int num_time_sample = 20;
    vector<float> time_seq;
    float seg_t0 = primitive_list[0][best_indices[0]].px.GetTimeInterval()[0];
    float seg_tf = primitive_list[0][best_indices[0]].px.GetTimeInterval()[1];
    for (int i = 0; i < num_time_sample; i++)
      time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) / (float)(num_time_sample - 1));
    geometry_msgs::msg::Point temp_point;
    for (int i = 0; i < best_indices.size(); i++) {
      line_strip.points.clear();
      line_strip.id = 0;
      line_strip.ns = std::to_string(i) + "-th best_target_prediction";
      for (int j = 0; j < num_time_sample; j++) {
        temp_point.x = primitive_list[i][best_indices[i]].px.GetValue(time_seq[j]);
        temp_point.y = primitive_list[i][best_indices[i]].py.GetValue(time_seq[j]);
        temp_point.z = primitive_list[i][best_indices[i]].pz.GetValue(time_seq[j]);
        line_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strip);
      visualization_msgs::msg::Marker erase_marker;
      //      erase_marker.type = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.ns = line_strip.ns;
      erase_marker.id = 1;
      erase_marker.header.frame_id = "map";
      visual_output.markers.push_back(erase_marker);
    }
  }
  return visual_output;
}
TargetRawPathVisualizationMsg
Visualizer::VisualizeRawTargetPathArray(const PrimitiveListSet &primitive_list) {
  TargetBestPathVisualizationMsg visual_output;
  bool is_primitive_generated_big = not primitive_list.empty();
  bool is_primitive_generated = true;
  if (is_primitive_generated_big) {
    for (int i = 0; i < primitive_list.size(); i++) {
      if (primitive_list[i].empty())
        is_primitive_generated = false;
    }
  }
  is_primitive_generated = is_primitive_generated and is_primitive_generated_big;
  if (is_primitive_generated) {
    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.color.a = 0.3;
    line_strip.color.r = 0.5;
    line_strip.color.g = 0.5;
    line_strip.color.b = 0.5;
    line_strip.scale.x = 0.01;
    line_strip.action = visualization_msgs::msg::Marker::MODIFY;
    line_strip.pose.orientation.w = 1.0;
    int num_time_sample = 20;
    vector<float> time_seq;
    float seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
    float seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
    for (int i = 0; i < num_time_sample; i++)
      time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) / (float)(num_time_sample - 1));
    geometry_msgs::msg::Point temp_point;
    int id = 0;
    for (int i = 0; i < primitive_list.size(); i++) {
      line_strip.ns = std::to_string(i) + "-th target_primitive";
      for (int j = 0; j < primitive_list[i].size(); j++) {
        line_strip.points.clear();
        line_strip.id = id;
        id++;
        for (int k = 0; k < num_time_sample; k++) {
          temp_point.x = primitive_list[i][j].px.GetValue(time_seq[k]);
          temp_point.y = primitive_list[i][j].py.GetValue(time_seq[k]);
          temp_point.z = primitive_list[i][j].pz.GetValue(time_seq[k]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
      visualization_msgs::msg::Marker erase_marker;
      erase_marker.type = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.ns = line_strip.ns;
      erase_marker.id = id;
      erase_marker.header.frame_id = "map";
      visual_output.markers.push_back(erase_marker);
    }
  }
  return visual_output;
}
TargetSafePathVisualizationMsg
Visualizer::VisualizeSafeTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexListSet &safe_indices) {
  TargetBestPathVisualizationMsg visual_output;

  bool is_primitive_generated_big = not primitive_list.empty();
  bool is_primitive_generated = true;
  if (is_primitive_generated_big) {
    for (int i = 0; i < primitive_list.size(); i++) {
      if (primitive_list[i].empty())
        is_primitive_generated = false;
    }
  }
  is_primitive_generated = is_primitive_generated and is_primitive_generated_big;
  bool is_safe_indices_big = not safe_indices.empty();
  bool is_safe_indices_generated = true;
  if (is_safe_indices_big) {
    for (int i = 0; i < safe_indices.size(); i++) {
      if (safe_indices[i].empty())
        is_safe_indices_generated = false;
    }
  }
  is_safe_indices_generated = is_safe_indices_generated and is_safe_indices_big;
  if (is_primitive_generated and is_safe_indices_generated) {
    visualization_msgs::msg::Marker line_strip;
    line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
    line_strip.header.frame_id = "map";
    line_strip.color.a = 0.5;
    line_strip.color.r = 0.8;
    line_strip.color.g = 0.2;
    line_strip.color.b = 0.0;
    line_strip.scale.x = 0.02;
    line_strip.action = visualization_msgs::msg::Marker::MODIFY;
    line_strip.pose.orientation.w = 1.0;
    int num_time_sample = 20;
    vector<float> time_seq;
    float seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
    float seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
    for (int i = 0; i < num_time_sample; i++)
      time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) / (float)(num_time_sample - 1));
    geometry_msgs::msg::Point temp_point;
    int id = 0;
    for (int i = 0; i < primitive_list.size(); i++) {
      line_strip.ns = std::to_string(i) + "-th safe_target_primitive";
      id = 0;
      for (int j = 0; j < safe_indices[i].size(); j++) {
        line_strip.points.clear();
        line_strip.id = id;
        id++;
        for (int k = 0; k < num_time_sample; k++) {
          temp_point.x = primitive_list[i][safe_indices[i][j]].px.GetValue(time_seq[k]);
          temp_point.y = primitive_list[i][safe_indices[i][j]].py.GetValue(time_seq[k]);
          temp_point.z = primitive_list[i][safe_indices[i][j]].pz.GetValue(time_seq[k]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
      visualization_msgs::msg::Marker erase_marker;
      erase_marker.type = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      erase_marker.ns = line_strip.ns;
      erase_marker.id = id;
      erase_marker.header.frame_id = "map";
      visual_output.markers.push_back(erase_marker);
    }
  }
  return visual_output;
}
