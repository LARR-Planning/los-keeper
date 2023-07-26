#include <los_keeper_ros2/visualization/visualizer.h>

using namespace los_keeper;

Visualizer::Visualizer() {}
Visualizer::Visualizer(const VisualizationParameters &parameters) : parameters_(parameters) {}
void Visualizer::UpdateTime(const rclcpp::Time &time) { time_ = time; }

ObstaclePathVisualizationMsg
Visualizer::VisualizeObstaclePathArray(const PrimitiveList &obstacle_primitive) {
  visualization_msgs::msg::MarkerArray visual_output;
  if (not parameters_.obstacle.publish)
    return visual_output;
  bool is_primitive_generated = not obstacle_primitive.empty();
  if (not is_primitive_generated)
    return visual_output;
  // prepare ros msgs
  std::vector<double> time_seq;
  double seg_t0 = obstacle_primitive[0].px.GetTimeInterval()[0];
  double seg_tf = obstacle_primitive[0].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.obstacle.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.obstacle.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  for (int i = 0; i < obstacle_primitive.size(); i++) { // the number of targets
    line_strips_.obstacle_path_strip.points.clear();
    line_strips_.obstacle_path_strip.id = i;
    for (int k = 0; k < parameters_.obstacle.num_time_sample; k++) {
      temp_point.x = obstacle_primitive[i].px.GetValue(time_seq[k]);
      temp_point.y = obstacle_primitive[i].py.GetValue(time_seq[k]);
      temp_point.z = obstacle_primitive[i].pz.GetValue(time_seq[k]);
      line_strips_.obstacle_path_strip.points.push_back(temp_point);
    }
    visual_output.markers.push_back(line_strips_.obstacle_path_strip);
  }
  return visual_output;
}
TargetBestPathVisualizationMsg
Visualizer::VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexList &best_indices) {
  TargetBestPathVisualizationMsg visual_output;
  if (not parameters_.target.best.publish)
    return visual_output;
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
  static int last_num_id = 0;
  if (not is_primitive_generated or not is_best_indices_generated) {
    visualization_msgs::msg::Marker erase_marker;
    for (int i = 0; i < last_num_id; i++) {
      erase_marker.action = visualization_msgs::msg::Marker::DELETE;
      erase_marker.ns = line_strips_.target_primitive_best_strip.ns;
      erase_marker.header.frame_id = parameters_.frame_id;
      erase_marker.id = i;
      visual_output.markers.push_back(erase_marker);
    }
    return visual_output;
  }
  // prepare ros msgs
  vector<double> time_seq;
  double seg_t0 = primitive_list[0][best_indices[0]].px.GetTimeInterval()[0];
  double seg_tf = primitive_list[0][best_indices[0]].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.target.best.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.target.best.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  for (int i = 0; i < best_indices.size(); i++) {
    line_strips_.target_primitive_best_strip.points.clear();
    line_strips_.target_primitive_best_strip.id = i;
    line_strips_.target_primitive_best_strip.ns = "best_target_prediction";
    for (int j = 0; j < parameters_.target.best.num_time_sample; j++) {
      temp_point.x = primitive_list[i][best_indices[i]].px.GetValue(time_seq[j]);
      temp_point.y = primitive_list[i][best_indices[i]].py.GetValue(time_seq[j]);
      temp_point.z = primitive_list[i][best_indices[i]].pz.GetValue(time_seq[j]);
      line_strips_.target_primitive_best_strip.points.push_back(temp_point);
    }
    visual_output.markers.push_back(line_strips_.target_primitive_best_strip);
  }
  last_num_id = best_indices.size();
  return visual_output;
}
TargetRawPathVisualizationMsg
Visualizer::VisualizeRawTargetPathArray(const PrimitiveListSet &primitive_list) {
  TargetBestPathVisualizationMsg visual_output;
  if (not parameters_.target.raw.publish)
    return visual_output;
  bool is_primitive_generated_big = not primitive_list.empty();
  bool is_primitive_generated = true;
  if (is_primitive_generated_big) {
    for (int i = 0; i < primitive_list.size(); i++) {
      if (primitive_list[i].empty())
        is_primitive_generated = false;
    }
  }
  is_primitive_generated = is_primitive_generated and is_primitive_generated_big;

  if (not is_primitive_generated)
    return visual_output;

  // prepare ros msgs
  vector<double> time_seq;
  double seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
  double seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.target.raw.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.target.raw.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  for (int i = 0; i < primitive_list.size(); i++) {
    int id = 0;
    int num_primitive_vis =
        int((float)primitive_list[i].size() * parameters_.target.raw.proportion);
    int index_jump = primitive_list[i].size() / num_primitive_vis;
    line_strips_.target_primitive_raw_strip.ns = std::to_string(i) + "-th target_primitive";
    for (int j = 0; j < num_primitive_vis - 1; j++) {
      line_strips_.target_primitive_raw_strip.points.clear();
      line_strips_.target_primitive_raw_strip.id = id;
      id++;
      for (int k = 0; k < parameters_.target.raw.num_time_sample; k++) {
        temp_point.x = primitive_list[i][index_jump * j].px.GetValue(time_seq[k]);
        temp_point.y = primitive_list[i][index_jump * j].py.GetValue(time_seq[k]);
        temp_point.z = primitive_list[i][index_jump * j].pz.GetValue(time_seq[k]);
        line_strips_.target_primitive_raw_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strips_.target_primitive_raw_strip);
    }
  }
  return visual_output;
}
TargetSafePathVisualizationMsg
Visualizer::VisualizeSafeTargetPathArray(const PrimitiveListSet &primitive_list,
                                         const IndexListSet &safe_indices) {
  TargetBestPathVisualizationMsg visual_output;
  if (not parameters_.target.safe.publish)
    return visual_output;
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
  static vector<int> last_num_id;

  if (not is_primitive_generated or not is_safe_indices_generated) {
    if (last_num_id.empty())
      return visual_output;
    for (int i = 0; i < last_num_id.size(); i++) {
      visualization_msgs::msg::Marker erase_marker;
      erase_marker.action = visualization_msgs::msg::Marker::DELETE;
      erase_marker.ns = std::to_string(i) + "-th safe_target_primitive";
      erase_marker.header.frame_id = parameters_.frame_id;
      for (int j = 0; j < last_num_id[i]; j++) {
        erase_marker.id = j + 1;
        visual_output.markers.push_back(erase_marker);
      }
      last_num_id[i] = 0;
    }
    return visual_output;
  }

  vector<double> time_seq;
  double seg_t0 = primitive_list[0][0].px.GetTimeInterval()[0];
  double seg_tf = primitive_list[0][0].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.target.safe.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.target.safe.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  for (int i = 0; i < primitive_list.size(); i++) {
    line_strips_.target_primitive_safe_strip.ns = std::to_string(i) + "-th safe_target_primitive";
    int id = 0;
    int num_safe_primitive_vis =
        int((float)safe_indices[i].size() * parameters_.target.raw.proportion);
    int index_jump = (int)safe_indices[i].size() / num_safe_primitive_vis;
    for (int j = 0; j < num_safe_primitive_vis - 1; j++) {
      line_strips_.target_primitive_safe_strip.points.clear();
      line_strips_.target_primitive_safe_strip.id = ++id;
      for (int k = 0; k < parameters_.target.safe.num_time_sample; k++) {
        temp_point.x = primitive_list[i][safe_indices[i][index_jump * j]].px.GetValue(time_seq[k]);
        temp_point.y = primitive_list[i][safe_indices[i][index_jump * j]].py.GetValue(time_seq[k]);
        temp_point.z = primitive_list[i][safe_indices[i][index_jump * j]].pz.GetValue(time_seq[k]);
        line_strips_.target_primitive_safe_strip.points.push_back(temp_point);
      }
      visual_output.markers.push_back(line_strips_.target_primitive_safe_strip);
    }
    if (last_num_id.empty() or last_num_id.size() < safe_indices.size()) {
      last_num_id.push_back(line_strips_.target_primitive_safe_strip.id);
    } else {
      visualization_msgs::msg::Marker erase_marker;
      erase_marker.action = visualization_msgs::msg::Marker::DELETE;
      erase_marker.ns = line_strips_.target_primitive_safe_strip.ns;
      erase_marker.header.frame_id = parameters_.frame_id;
      for (int j = line_strips_.target_primitive_safe_strip.id + 1; j <= last_num_id[i]; j++) {
        erase_marker.id = j;
        visual_output.markers.push_back(erase_marker);
      }
      last_num_id[i] = line_strips_.target_primitive_safe_strip.id;
    }
  }
  return visual_output;
}
KeeperRawPathVisualizationMsg
Visualizer::VisualizeRawKeeperPathArray(const PrimitiveList &primitive_list) {
  KeeperRawPathVisualizationMsg visual_output;
  if (not parameters_.keeper.raw.publish)
    return visual_output;
  if (primitive_list.empty())
    return visual_output;
  // prepare ros msgs
  vector<double> time_seq;
  double seg_t0 = primitive_list[0].px.GetTimeInterval()[0];
  double seg_tf = primitive_list[0].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.keeper.raw.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.keeper.raw.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  int id = 0;
  int num_primitive_vis = int((float)primitive_list.size() * parameters_.keeper.raw.proportion);
  int index_jump = primitive_list.size() / num_primitive_vis;
  for (int j = 0; j < num_primitive_vis - 1; j++) {
    line_strips_.keeper_primitive_raw_strip.points.clear();
    line_strips_.keeper_primitive_raw_strip.id = id;
    id++;
    for (int k = 0; k < parameters_.target.raw.num_time_sample; k++) {
      temp_point.x = primitive_list[index_jump * j].px.GetValue(time_seq[k]);
      temp_point.y = primitive_list[index_jump * j].py.GetValue(time_seq[k]);
      temp_point.z = primitive_list[index_jump * j].pz.GetValue(time_seq[k]);
      line_strips_.keeper_primitive_raw_strip.points.push_back(temp_point);
    }
    visual_output.markers.push_back(line_strips_.keeper_primitive_raw_strip);
  }
  return visual_output;
}
KeeperSafePathVisualizationMsg
Visualizer::VisualizeSafeKeeperPathArray(const PrimitiveList &primitive_list,
                                         const IndexList &safe_indices) {
  KeeperSafePathVisualizationMsg visual_output;
  if (not parameters_.keeper.safe.publish)
    return visual_output;

  static int last_num_id = -1;
  if (primitive_list.empty() or safe_indices.empty()) {
    if (last_num_id < 0)
      return visual_output;
    visualization_msgs::msg::Marker erase_marker;
    erase_marker.action = visualization_msgs::msg::Marker::DELETE;
    erase_marker.ns = line_strips_.keeper_primitive_safe_strip.ns;
    erase_marker.header.frame_id = parameters_.frame_id;
    for (int i = 0; i < last_num_id; i++) {
      erase_marker.id = i + 1;
      visual_output.markers.push_back(erase_marker);
    }
    last_num_id = 0;
    return visual_output;
  }

  vector<double> time_seq;
  double seg_t0 = primitive_list[0].px.GetTimeInterval()[0];
  double seg_tf = primitive_list[0].px.GetTimeInterval()[1];
  for (int i = 0; i < parameters_.keeper.safe.num_time_sample; i++)
    time_seq.push_back(seg_t0 + (double)i * (seg_tf - seg_t0) /
                                    (double)(parameters_.keeper.raw.num_time_sample - 1));
  geometry_msgs::msg::Point temp_point;
  int id = 0;
  int num_primitive_vis = int((float)safe_indices.size() * parameters_.keeper.raw.proportion);
  int index_jump = (int)safe_indices.size() / num_primitive_vis;
  for (int j = 0; j < num_primitive_vis - 1; j++) {
    line_strips_.keeper_primitive_safe_strip.points.clear();
    line_strips_.keeper_primitive_safe_strip.id = ++id;
    for (int k = 0; k < parameters_.target.safe.num_time_sample; k++) {
      temp_point.x = primitive_list[safe_indices[index_jump * j]].px.GetValue(time_seq[k]);
      temp_point.y = primitive_list[safe_indices[index_jump * j]].py.GetValue(time_seq[k]);
      temp_point.z = primitive_list[safe_indices[index_jump * j]].pz.GetValue(time_seq[k]);
      line_strips_.keeper_primitive_safe_strip.points.push_back(temp_point);
    }
    visual_output.markers.push_back(line_strips_.keeper_primitive_safe_strip);
  }
  if (last_num_id < 0) {
    last_num_id = line_strips_.keeper_primitive_safe_strip.id;
  } else {
    visualization_msgs::msg::Marker erase_marker;
    erase_marker.action = visualization_msgs::msg::Marker::DELETE;
    erase_marker.ns = line_strips_.keeper_primitive_safe_strip.ns;
    erase_marker.header.frame_id = parameters_.frame_id;
    for (int j = line_strips_.keeper_primitive_safe_strip.id + 1; j <= last_num_id; j++) {
      erase_marker.id = j;
      visual_output.markers.push_back(erase_marker);
    }
    last_num_id = line_strips_.keeper_primitive_safe_strip.id;
  }
  return visual_output;
}
FailVisualizationMsg Visualizer::VisualizeFailFlagList(const bool &success_flag_prediction,
                                                       const bool &success_flag_planning,
                                                       const uint &seq) {
  FailVisualizationMsg visual_output;
  if (seq == 0)
    return visual_output;
  static int last_num_id = 0;
  int num_id = 0;
  if (not success_flag_prediction) {
    fail_markers_.fail_flag_prediction.id = num_id++;
    visual_output.markers.push_back(fail_markers_.fail_flag_prediction);
  }
  if (not success_flag_planning and success_flag_prediction) {
    fail_markers_.fail_flag_planning.id = num_id++;
    visual_output.markers.push_back(fail_markers_.fail_flag_planning);
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
void Visualizer::UpdateParam(const VisualizationParameters &param) {
  parameters_ = param;
  // obstacle path
  line_strips_.obstacle_path_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.obstacle_path_strip.header.frame_id = parameters_.frame_id;
  line_strips_.obstacle_path_strip.color.a = parameters_.obstacle.color.a;
  line_strips_.obstacle_path_strip.color.r = parameters_.obstacle.color.r;
  line_strips_.obstacle_path_strip.color.g = parameters_.obstacle.color.g;
  line_strips_.obstacle_path_strip.color.b = parameters_.obstacle.color.b;
  line_strips_.obstacle_path_strip.scale.x = parameters_.obstacle.line_scale;
  line_strips_.obstacle_path_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.obstacle_path_strip.pose.orientation.w = 1.0;
  line_strips_.obstacle_path_strip.ns = "obstacle_path";
  // target raw primitive
  line_strips_.target_primitive_raw_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.target_primitive_raw_strip.header.frame_id = parameters_.frame_id;
  line_strips_.target_primitive_raw_strip.color.a = parameters_.target.raw.color.a;
  line_strips_.target_primitive_raw_strip.color.r = parameters_.target.raw.color.r;
  line_strips_.target_primitive_raw_strip.color.g = parameters_.target.raw.color.g;
  line_strips_.target_primitive_raw_strip.color.b = parameters_.target.raw.color.b;
  line_strips_.target_primitive_raw_strip.scale.x = parameters_.target.raw.line_scale;
  line_strips_.target_primitive_raw_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.target_primitive_raw_strip.pose.orientation.w = 1.0;
  // target safe primitive
  line_strips_.target_primitive_safe_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.target_primitive_safe_strip.header.frame_id = parameters_.frame_id;
  line_strips_.target_primitive_safe_strip.color.a = parameters_.target.safe.color.a;
  line_strips_.target_primitive_safe_strip.color.r = parameters_.target.safe.color.r;
  line_strips_.target_primitive_safe_strip.color.g = parameters_.target.safe.color.g;
  line_strips_.target_primitive_safe_strip.color.b = parameters_.target.safe.color.b;
  line_strips_.target_primitive_safe_strip.scale.x = parameters_.target.safe.line_scale;
  line_strips_.target_primitive_safe_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.target_primitive_safe_strip.pose.orientation.w = 1.0;
  // target best primitive
  line_strips_.target_primitive_best_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.target_primitive_best_strip.header.frame_id = parameters_.frame_id;
  line_strips_.target_primitive_best_strip.color.a = parameters_.target.best.color.a;
  line_strips_.target_primitive_best_strip.color.r = parameters_.target.best.color.r;
  line_strips_.target_primitive_best_strip.color.g = parameters_.target.best.color.g;
  line_strips_.target_primitive_best_strip.color.b = parameters_.target.best.color.b;
  line_strips_.target_primitive_best_strip.scale.x = parameters_.target.best.line_scale;
  line_strips_.target_primitive_best_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.target_primitive_best_strip.pose.orientation.w = 1.0;
  // keeper raw primitive
  line_strips_.keeper_primitive_raw_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.keeper_primitive_raw_strip.header.frame_id = parameters_.frame_id;
  line_strips_.keeper_primitive_raw_strip.color.a = parameters_.keeper.raw.color.a;
  line_strips_.keeper_primitive_raw_strip.color.r = parameters_.keeper.raw.color.r;
  line_strips_.keeper_primitive_raw_strip.color.g = parameters_.keeper.raw.color.g;
  line_strips_.keeper_primitive_raw_strip.color.b = parameters_.keeper.raw.color.b;
  line_strips_.keeper_primitive_raw_strip.scale.x = parameters_.keeper.raw.line_scale;
  line_strips_.keeper_primitive_raw_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.keeper_primitive_raw_strip.pose.orientation.w = 1.0;
  line_strips_.keeper_primitive_raw_strip.ns = "keeper_primitives";
  // keeper safe primitive
  line_strips_.keeper_primitive_safe_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.keeper_primitive_safe_strip.header.frame_id = parameters_.frame_id;
  line_strips_.keeper_primitive_safe_strip.color.a = parameters_.keeper.safe.color.a;
  line_strips_.keeper_primitive_safe_strip.color.r = parameters_.keeper.safe.color.r;
  line_strips_.keeper_primitive_safe_strip.color.g = parameters_.keeper.safe.color.g;
  line_strips_.keeper_primitive_safe_strip.color.b = parameters_.keeper.safe.color.b;
  line_strips_.keeper_primitive_safe_strip.scale.x = parameters_.keeper.safe.line_scale;
  line_strips_.keeper_primitive_safe_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.keeper_primitive_safe_strip.pose.orientation.w = 1.0;
  line_strips_.keeper_primitive_safe_strip.ns = "keeper_safe_primitives";
  // keeper best primitive
  line_strips_.keeper_primitive_best_strip.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_strips_.keeper_primitive_best_strip.header.frame_id = parameters_.frame_id;
  line_strips_.keeper_primitive_best_strip.color.a = parameters_.keeper.best.color.a;
  line_strips_.keeper_primitive_best_strip.color.r = parameters_.keeper.best.color.r;
  line_strips_.keeper_primitive_best_strip.color.g = parameters_.keeper.best.color.g;
  line_strips_.keeper_primitive_best_strip.color.b = parameters_.keeper.best.color.b;
  line_strips_.keeper_primitive_best_strip.scale.x = parameters_.keeper.best.line_scale;
  line_strips_.keeper_primitive_best_strip.action = visualization_msgs::msg::Marker::MODIFY;
  line_strips_.keeper_primitive_best_strip.pose.orientation.w = 1.0;
  line_strips_.keeper_primitive_best_strip.ns = "keeper_best_primitives";
  // Fail flag: planning
  fail_markers_.fail_flag_planning.type = visualization_msgs::msg::Marker::CUBE;
  fail_markers_.fail_flag_planning.header.frame_id = parameters_.frame_id;
  fail_markers_.fail_flag_planning.color.a = parameters_.fail_flag.planning.color.a;
  fail_markers_.fail_flag_planning.color.r = parameters_.fail_flag.planning.color.r;
  fail_markers_.fail_flag_planning.color.g = parameters_.fail_flag.planning.color.g;
  fail_markers_.fail_flag_planning.color.b = parameters_.fail_flag.planning.color.b;
  fail_markers_.fail_flag_planning.scale.x = 1000.0;
  fail_markers_.fail_flag_planning.scale.y = 1000.0;
  fail_markers_.fail_flag_planning.scale.z = 100.0;
  fail_markers_.fail_flag_planning.pose.position.x = 0.0;
  fail_markers_.fail_flag_planning.pose.position.y = 0.0;
  fail_markers_.fail_flag_planning.pose.position.z = 0.0;
  fail_markers_.fail_flag_planning.pose.orientation.w = 1.0;
  fail_markers_.fail_flag_planning.pose.orientation.x = 0.0;
  fail_markers_.fail_flag_planning.pose.orientation.y = 0.0;
  fail_markers_.fail_flag_planning.pose.orientation.z = 0.0;
  fail_markers_.fail_flag_planning.action = visualization_msgs::msg::Marker::ADD;
  fail_markers_.fail_flag_planning.ns = "fail_flag";
  // Fail flag: prediction
  fail_markers_.fail_flag_prediction.type = visualization_msgs::msg::Marker::CUBE;
  fail_markers_.fail_flag_prediction.header.frame_id = parameters_.frame_id;
  fail_markers_.fail_flag_prediction.color.a = parameters_.fail_flag.prediction.color.a;
  fail_markers_.fail_flag_prediction.color.r = parameters_.fail_flag.prediction.color.r;
  fail_markers_.fail_flag_prediction.color.g = parameters_.fail_flag.prediction.color.g;
  fail_markers_.fail_flag_prediction.color.b = parameters_.fail_flag.prediction.color.b;
  fail_markers_.fail_flag_prediction.scale.x = 1000.0;
  fail_markers_.fail_flag_prediction.scale.y = 1000.0;
  fail_markers_.fail_flag_prediction.scale.z = 100.0;
  fail_markers_.fail_flag_prediction.pose.position.x = 0.0;
  fail_markers_.fail_flag_prediction.pose.position.y = 0.0;
  fail_markers_.fail_flag_prediction.pose.position.z = 0.0;
  fail_markers_.fail_flag_prediction.pose.orientation.w = 1.0;
  fail_markers_.fail_flag_prediction.pose.orientation.x = 0.0;
  fail_markers_.fail_flag_prediction.pose.orientation.y = 0.0;
  fail_markers_.fail_flag_prediction.pose.orientation.z = 0.0;
  fail_markers_.fail_flag_prediction.action = visualization_msgs::msg::Marker::ADD;
  fail_markers_.fail_flag_prediction.ns = "fail_flag";
}
