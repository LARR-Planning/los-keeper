#ifndef HEADER_VISUALIZER
#define HEADER_VISUALIZER

#include "los_keeper/wrapper/wrapper.h"

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using ObstaclePathVisualizationMsg = visualization_msgs::msg::MarkerArray;
using TargetBestPathVisualizationMsg = visualization_msgs::msg::MarkerArray;
using TargetSafePathVisualizationMsg = visualization_msgs::msg::MarkerArray;
using TargetRawPathVisualizationMsg = visualization_msgs::msg::MarkerArray;

namespace los_keeper {
struct VisualizationParameters {
  struct {
    float color_r{0.3};
    float color_g{0.3};
    float color_b{0.3};
    float color_a{0.2};
    float scale_x{0.01};
  } structured_obstacle_path;
  double marker_size{1.2};
  string frame_id;
};

class Visualizer {
  VisualizationParameters parameters_;
  rclcpp::Time time_;

public:
  Visualizer();
  Visualizer(const VisualizationParameters &parameters);
  void UpdateTime(const rclcpp::Time &time);
  visualization_msgs::msg::Marker DeriveSomeDebugInfo(const int some_debug_info) const;
  ObstaclePathVisualizationMsg VisualizeObstaclePathArray(const PrimitiveList &obstacle_primitive);
  TargetBestPathVisualizationMsg
  VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                               const IndexList &best_indices);
  TargetSafePathVisualizationMsg
  VisualizeSafeTargetPathArray(const PrimitiveListSet &primitive_list,
                               const IndexListSet &safe_indices);
  TargetRawPathVisualizationMsg VisualizeRawTargetPathArray(const PrimitiveListSet &primitive_list);
};

} // namespace los_keeper

#endif /* HEADER_VISUALIZER */
