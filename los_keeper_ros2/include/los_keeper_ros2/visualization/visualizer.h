#ifndef HEADER_VISUALIZER
#define HEADER_VISUALIZER

#include "los_keeper/wrapper/wrapper.h"

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace los_keeper {
struct VisualizationParameters {
  double marker_size{1.2};
};

class Visualizer {
  VisualizationParameters parameters_;
  rclcpp::Time time_;

public:
  Visualizer();
  Visualizer(const VisualizationParameters &parameters);
  void UpdateTime(const rclcpp::Time &time);
  visualization_msgs::msg::Marker DeriveSomeDebugInfo(const int some_debug_info) const;
};

} // namespace los_keeper

#endif /* HEADER_VISUALIZER */
