#include <los_keeper_ros2/visualization/visualizer.h>

using namespace los_keeper;

Visualizer::Visualizer() {}
Visualizer::Visualizer(const VisualizationParameters &parameters) : parameters_(parameters) {}
void Visualizer::UpdateTime(const rclcpp::Time &time) { time_ = time; }

ObstaclePathVisualizationMsg
Visualizer::VisualizeObstaclePathArray(const PrimitiveList &obstacle_primitive) {
  visualization_msgs::msg::MarkerArray visual_output;
  if (parameters_.obstacle.publish) {
    bool is_primitive_generated = not obstacle_primitive.empty();
    if (is_primitive_generated) {
      visualization_msgs::msg::Marker line_strip;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.obstacle.color.a;
      line_strip.color.r = parameters_.obstacle.color.r;
      line_strip.color.g = parameters_.obstacle.color.g;
      line_strip.color.b = parameters_.obstacle.color.b;
      line_strip.scale.x = parameters_.obstacle.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;
      line_strip.ns = "obstacle_path";
      std::vector<float> time_seq;
      float seg_t0 = obstacle_primitive[0].px.GetTimeInterval()[0];
      float seg_tf = obstacle_primitive[0].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.obstacle.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.obstacle.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      for (int i = 0; i < obstacle_primitive.size(); i++) { // the number of targets
        line_strip.points.clear();
        line_strip.id = i;
        for (int k = 0; k < parameters_.obstacle.num_time_sample; k++) {
          temp_point.x = obstacle_primitive[i].px.GetValue(time_seq[k]);
          temp_point.y = obstacle_primitive[i].py.GetValue(time_seq[k]);
          temp_point.z = obstacle_primitive[i].pz.GetValue(time_seq[k]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
    }
  }
  return visual_output;
}
TargetBestPathVisualizationMsg
Visualizer::VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexList &best_indices) {
  TargetBestPathVisualizationMsg visual_output;
  if (parameters_.target.best.publish) {
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
      static vector<int> last_num_id;
      visualization_msgs::msg::Marker line_strip;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.target.best.color.a;
      line_strip.color.r = parameters_.target.best.color.r;
      line_strip.color.g = parameters_.target.best.color.g;
      line_strip.color.b = parameters_.target.best.color.b;
      line_strip.scale.x = parameters_.target.best.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;

      vector<float> time_seq;
      float seg_t0 = primitive_list[0][best_indices[0]].px.GetTimeInterval()[0];
      float seg_tf = primitive_list[0][best_indices[0]].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.target.best.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.target.best.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      for (int i = 0; i < best_indices.size(); i++) {
        line_strip.points.clear();
        line_strip.id = 0;
        line_strip.ns = std::to_string(i) + "-th best_target_prediction";
        for (int j = 0; j < parameters_.target.best.num_time_sample; j++) {
          temp_point.x = primitive_list[i][best_indices[i]].px.GetValue(time_seq[j]);
          temp_point.y = primitive_list[i][best_indices[i]].py.GetValue(time_seq[j]);
          temp_point.z = primitive_list[i][best_indices[i]].pz.GetValue(time_seq[j]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
    }
  }
  return visual_output;
}
TargetRawPathVisualizationMsg
Visualizer::VisualizeRawTargetPathArray(const PrimitiveListSet &primitive_list) {
  TargetBestPathVisualizationMsg visual_output;
  if (parameters_.target.raw.publish) {
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
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.target.raw.color.a;
      line_strip.color.r = parameters_.target.raw.color.r;
      line_strip.color.g = parameters_.target.raw.color.g;
      line_strip.color.b = parameters_.target.raw.color.b;
      line_strip.scale.x = parameters_.target.raw.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;
      vector<float> time_seq;
      float seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
      float seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.target.raw.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.target.raw.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      for (int i = 0; i < primitive_list.size(); i++) {
        int id = 0;
        int num_primitive_vis =
            int((float)primitive_list[i].size() * parameters_.target.raw.proportion);
        int index_jump = primitive_list[i].size() / num_primitive_vis;
        line_strip.ns = std::to_string(i) + "-th target_primitive";
        for (int j = 0; j < num_primitive_vis - 1; j++) {
          line_strip.points.clear();
          line_strip.id = id;
          id++;
          for (int k = 0; k < parameters_.target.raw.num_time_sample; k++) {
            temp_point.x = primitive_list[i][index_jump * j].px.GetValue(time_seq[k]);
            temp_point.y = primitive_list[i][index_jump * j].py.GetValue(time_seq[k]);
            temp_point.z = primitive_list[i][index_jump * j].pz.GetValue(time_seq[k]);
            line_strip.points.push_back(temp_point);
          }
          visual_output.markers.push_back(line_strip);
        }
      }
    }
  }
  return visual_output;
}
TargetSafePathVisualizationMsg
Visualizer::VisualizeSafeTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexListSet &safe_indices) {
  TargetBestPathVisualizationMsg visual_output;
  if (parameters_.target.safe.publish) {
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
      static vector<int> last_num_id;
      visualization_msgs::msg::Marker line_strip;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.target.safe.color.a;
      line_strip.color.r = parameters_.target.safe.color.r;
      line_strip.color.g = parameters_.target.safe.color.g;
      line_strip.color.b = parameters_.target.safe.color.b;
      line_strip.scale.x = parameters_.target.safe.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;
      vector<float> time_seq;
      float seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
      float seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.target.safe.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.target.safe.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      for (int i = 0; i < primitive_list.size(); i++) {
        line_strip.ns = std::to_string(i) + "-th safe_target_primitive";
        int id = 0;
        int num_safe_primitive_vis =
            int((float)safe_indices[i].size() * parameters_.target.raw.proportion);
        int index_jump = (int)safe_indices[i].size() / num_safe_primitive_vis;
        for (int j = 0; j < num_safe_primitive_vis - 1; j++) {
          line_strip.points.clear();
          line_strip.id = ++id;
          for (int k = 0; k < parameters_.target.safe.num_time_sample; k++) {
            temp_point.x =
                primitive_list[i][safe_indices[i][index_jump * j]].px.GetValue(time_seq[k]);
            temp_point.y =
                primitive_list[i][safe_indices[i][index_jump * j]].py.GetValue(time_seq[k]);
            temp_point.z =
                primitive_list[i][safe_indices[i][index_jump * j]].pz.GetValue(time_seq[k]);
            line_strip.points.push_back(temp_point);
          }
          visual_output.markers.push_back(line_strip);
        }
        if (last_num_id.empty() or last_num_id.size() < safe_indices.size()) {
          last_num_id.push_back(line_strip.id);
        } else {
          visualization_msgs::msg::Marker erase_marker;
          erase_marker.action = visualization_msgs::msg::Marker::DELETE;
          erase_marker.ns = line_strip.ns;
          erase_marker.header.frame_id = parameters_.frame_id;
          for (int j = line_strip.id + 1; j <= last_num_id[i]; j++) {
            erase_marker.id = j;
            visual_output.markers.push_back(erase_marker);
          }
          last_num_id[i] = line_strip.id;
        }
      }
    }
  }
  return visual_output;
}
KeeperRawPathVisualizationMsg
Visualizer::VisualizeRawKeeperPathArray(const PrimitiveList &primitive_list) {
  KeeperRawPathVisualizationMsg visual_output;
  if (parameters_.keeper.raw.publish) {
    //    printf("PUBLISH KEEPER RAW PUBLISH \n");
    if (not primitive_list.empty()) {
      visualization_msgs::msg::Marker line_strip;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.keeper.raw.color.a;
      line_strip.color.r = parameters_.keeper.raw.color.r;
      line_strip.color.g = parameters_.keeper.raw.color.g;
      line_strip.color.b = parameters_.keeper.raw.color.b;
      line_strip.scale.x = parameters_.keeper.raw.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;
      vector<double> time_seq;
      double seg_t0 = primitive_list[0].px.GetTimeInterval()[0];
      double seg_tf = primitive_list[0].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.keeper.raw.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.keeper.raw.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      int id = 0;
      int num_primitive_vis = int((float)primitive_list.size() * parameters_.keeper.raw.proportion);
      int index_jump = primitive_list.size() / num_primitive_vis;
      line_strip.ns = "keeper_primitives";
      for (int j = 0; j < num_primitive_vis - 1; j++) {
        line_strip.points.clear();
        line_strip.id = id;
        id++;
        for (int k = 0; k < parameters_.target.raw.num_time_sample; k++) {
          temp_point.x = primitive_list[index_jump * j].px.GetValue(time_seq[k]);
          temp_point.y = primitive_list[index_jump * j].py.GetValue(time_seq[k]);
          temp_point.z = primitive_list[index_jump * j].pz.GetValue(time_seq[k]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
    }
  }
  return visual_output;
}
KeeperSafePathVisualizationMsg
Visualizer::VisualizeSafeKeeperPathArray(const PrimitiveList &primitive_list,
                                         const IndexList &safe_indices) {
  KeeperSafePathVisualizationMsg visual_output;
  if (parameters_.keeper.safe.publish) {
    if (not primitive_list.empty() and not safe_indices.empty()) {
      static int last_num_id;
      visualization_msgs::msg::Marker line_strip;
      line_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
      line_strip.header.frame_id = parameters_.frame_id;
      line_strip.color.a = parameters_.keeper.safe.color.a;
      line_strip.color.r = parameters_.keeper.safe.color.r;
      line_strip.color.g = parameters_.keeper.safe.color.g;
      line_strip.color.b = parameters_.keeper.safe.color.b;
      line_strip.scale.x = parameters_.keeper.safe.line_scale;
      line_strip.action = visualization_msgs::msg::Marker::MODIFY;
      line_strip.pose.orientation.w = 1.0;
      vector<double> time_seq;
      double seg_t0 = primitive_list[0].px.GetTimeInterval()[0];
      double seg_tf = primitive_list[0].px.GetTimeInterval()[1];
      for (int i = 0; i < parameters_.keeper.safe.num_time_sample; i++)
        time_seq.push_back(seg_t0 + (float)i * (seg_tf - seg_t0) /
                                        (float)(parameters_.keeper.raw.num_time_sample - 1));
      geometry_msgs::msg::Point temp_point;
      int id = 0;
      int num_primitive_vis = int((float)safe_indices.size() * parameters_.keeper.raw.proportion);
      int index_jump = (int)safe_indices.size() / num_primitive_vis;
      line_strip.ns = "keeper_safe_primitives";
      for (int j = 0; j < num_primitive_vis - 1; j++) {
        line_strip.points.clear();
        line_strip.id = ++id;
        for (int k = 0; k < parameters_.target.safe.num_time_sample; k++) {
          temp_point.x = primitive_list[safe_indices[index_jump * j]].px.GetValue(time_seq[k]);
          temp_point.y = primitive_list[safe_indices[index_jump * j]].py.GetValue(time_seq[k]);
          temp_point.z = primitive_list[safe_indices[index_jump * j]].pz.GetValue(time_seq[k]);
          line_strip.points.push_back(temp_point);
        }
        visual_output.markers.push_back(line_strip);
      }
      if (last_num_id < 0) {
        last_num_id = line_strip.id;
      } else {
        visualization_msgs::msg::Marker erase_marker;
        erase_marker.action = visualization_msgs::msg::Marker::DELETE;
        erase_marker.ns = line_strip.ns;
        erase_marker.header.frame_id = parameters_.frame_id;
        for (int j = line_strip.id + 1; j <= last_num_id; j++) {
          erase_marker.id = j;
          visual_output.markers.push_back(erase_marker);
        }
        last_num_id = line_strip.id;
      }
    }
  }
  return visual_output;
}
FailVisualizationMsg Visualizer::VisualizeFailFlagList(const bool &success_flag_prediction,
                                                       const bool &success_flag_planning) {
  FailVisualizationMsg visual_output;
  static int last_num_id = 0;
  int num_id = 0;
  if (not success_flag_planning) {
    visualization_msgs::msg::Marker fail_marker;
    fail_marker.type = visualization_msgs::msg::Marker::CUBE;
    fail_marker.header.frame_id = parameters_.frame_id;
    fail_marker.color.a = parameters_.fail_flag.planning.color.a;
    fail_marker.color.r = parameters_.fail_flag.planning.color.r;
    fail_marker.color.g = parameters_.fail_flag.planning.color.g;
    fail_marker.color.b = parameters_.fail_flag.planning.color.b;
    fail_marker.scale.x = 100.0;
    fail_marker.scale.y = 100.0;
    fail_marker.scale.z = 100.0;
    fail_marker.action = visualization_msgs::msg::Marker::ADD;
    fail_marker.pose.position.x = 0.0;
    fail_marker.pose.position.y = 0.0;
    fail_marker.pose.position.z = 0.0;
    fail_marker.pose.orientation.w = 1.0;
    fail_marker.pose.orientation.x = 0.0;
    fail_marker.pose.orientation.y = 0.0;
    fail_marker.pose.orientation.z = 0.0;
    fail_marker.ns = "fail_flag";
    fail_marker.id = num_id++;
    visual_output.markers.push_back(fail_marker);
  }
  if (not success_flag_prediction) {
    printf("FAIL PREDICTION \n");
    visualization_msgs::msg::Marker fail_marker;
    fail_marker.type = visualization_msgs::msg::Marker::CUBE;
    fail_marker.header.frame_id = parameters_.frame_id;
    fail_marker.color.a = parameters_.fail_flag.prediction.color.a;
    fail_marker.color.r = parameters_.fail_flag.prediction.color.r;
    fail_marker.color.g = parameters_.fail_flag.prediction.color.g;
    fail_marker.color.b = parameters_.fail_flag.prediction.color.b;
    fail_marker.scale.x = 100.0;
    fail_marker.scale.y = 100.0;
    fail_marker.scale.z = 100.0;
    fail_marker.action = visualization_msgs::msg::Marker::ADD;
    fail_marker.pose.position.x = 0.0;
    fail_marker.pose.position.y = 0.0;
    fail_marker.pose.position.z = 0.0;
    fail_marker.pose.orientation.w = 1.0;
    fail_marker.pose.orientation.x = 0.0;
    fail_marker.pose.orientation.y = 0.0;
    fail_marker.pose.orientation.z = 0.0;
    fail_marker.ns = "fail_flag";
    fail_marker.id = num_id++;
    visual_output.markers.push_back(fail_marker);
  }
  visualization_msgs::msg::Marker erase_marker;
  erase_marker.action = visualization_msgs::msg::Marker::DELETE;
  erase_marker.ns = "fail_flag";
  erase_marker.header.frame_id = parameters_.frame_id;
  for (int i = num_id; i < last_num_id; i++) {
    erase_marker.id = i;
    visual_output.markers.push_back(erase_marker);
  }
  last_num_id = num_id;
  return visual_output;
}
