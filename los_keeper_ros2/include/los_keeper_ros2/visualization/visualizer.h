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
using KeeperRawPathVisualizationMsg = visualization_msgs::msg::MarkerArray;
using KeeperSafePathVisualizationMsg = visualization_msgs::msg::MarkerArray;
using KeeperDyanmicallyFeasiblePathVisualizationMsg = visualization_msgs::msg::MarkerArray;

namespace los_keeper {
struct VisualizationParameters {
  struct {
    bool publish{true};
    int num_time_sample{2};
    float line_scale{0.01};
    struct {
      float a{0.3};
      float r{0.5};
      float g{0.5};
      float b{0.5};
    } color;
  } obstacle;
  struct {
    struct {
      bool publish{true};
      float proportion{0.5};
      int num_time_sample{5};
      struct {
        float a{0.3};
        float r{0.5};
        float g{0.5};
        float b{0.5};
      } color;
      float line_scale{0.01};
    } raw;
    struct {
      bool publish{true};
      float proportion{0.5};
      int num_time_sample{5};
      struct {
        float a{0.3};
        float r{1.0};
        float g{0.0};
        float b{1.0};
      } color;
      float line_scale{0.02};
    } safe;
    struct {
      bool publish{true};
      int num_time_sample{5};
      struct {
        float a{0.3};
        float r{1.0};
        float g{0.0};
        float b{0.0};
      } color;
      float line_scale{0.02};
    } best;
  } target;
  struct {
    struct {
      bool publish{true};
      float proportion{0.5};
      int num_time_sample{5};
      struct {
        float a{0.1};
        float r{0.0};
        float g{1.0};
        float b{1.0};
      } color;
      float line_scale{0.01};
    } raw;
    struct {
      bool publish{true};
      float proportion{0.5};
      int num_time_sample{5};
      struct {
        float a{0.5};
        float r{0.5};
        float g{0.5};
        float b{1.0};
      } color;
      float line_scale{0.01};
    } safe;
  } keeper;
  string frame_id{"map"};
};

class Visualizer {
  VisualizationParameters parameters_;
  rclcpp::Time time_;

public:
  Visualizer();
  Visualizer(const VisualizationParameters &parameters);
  void UpdateTime(const rclcpp::Time &time);
  void UpdateParam(const VisualizationParameters &param) { parameters_ = param; };
  ObstaclePathVisualizationMsg VisualizeObstaclePathArray(const PrimitiveList &obstacle_primitive);
  TargetBestPathVisualizationMsg
  VisualizeBestTargetPathArray(const PrimitiveListSet &primitive_list,
                               const IndexList &best_indices);
  TargetSafePathVisualizationMsg
  VisualizeSafeTargetPathArray(const PrimitiveListSet &primitive_list,
                               const IndexListSet &safe_indices);
  TargetRawPathVisualizationMsg VisualizeRawTargetPathArray(const PrimitiveListSet &primitive_list);
  KeeperRawPathVisualizationMsg VisualizeRawKeeperPathArray(const PrimitiveList &primitive_list);
  KeeperSafePathVisualizationMsg VisualizeSafeKeeperPathArray(const PrimitiveList &primitive_list,
                                                              const IndexList &safe_indices);
};

} // namespace los_keeper

#endif /* HEADER_VISUALIZER */
