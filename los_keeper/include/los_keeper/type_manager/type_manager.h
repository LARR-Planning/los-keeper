//
// Created by larr-planning on 23. 5. 16.
//
#ifndef LOS_KEEPER_TYPE_MANAGER_H
#define LOS_KEEPER_TYPE_MANAGER_H

#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include "thread"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace los_keeper {
typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

struct PredictionParam {
  int num_sample{0};                   // The number of primitives
  int num_thread{1};                   // The number of threads used in Prediction
  float planning_horizon{1.0f};        // The time horizon
  float acc_max{0.0f};                 // The maximum acceleration of the targets
  float detect_range{1.0f};            // Range of consideration of structured obstacles.
  float virtual_pcl_zone_width{1.0f};  // Range of virtual pcl to make SFC (width)
  float virtual_pcl_zone_height{1.0f}; // Range of virtual pcl to make SFC (height)
  bool is_lite{false}; // Lite vs Pro version, TODO: Only Pro version is implemented currently.
};

struct PlanningParam {
  int num_sample{0};                   // The number of primitives
  int num_thread{1};                   // The number of threads used in Planning
  float target_distance_min{1.0f};     // Minimum distance between the target and the drone.
  float target_distance_max{2.0f};     // Maximum distance between the target and the drone.
  float planning_horizon{1.0f};        // The time horizon
  float vel_max{0.0f};                 // Maximum velocity
  float acc_max{0.0f};                 // Maximum acceleartion
  float fov_angle{1.5707f};            // FOV angle
  float rx{0.01f};                     // Drone radius (x-axis)
  float ry{0.01f};                     // Drone radius (y-axis)
  float rz{0.01f};                     // Drone radius (z-axis)
  float virtual_pcl_zone_width{1.0f};  // Range of virtual pcl to make SFC (width)
  float virtual_pcl_zone_height{1.0f}; // Range of virtual pcl to make SFC (height)
  bool is_lite{false}; // Lite vs Pro version, TODO: Only Pro version is implemented currently.
};

struct ProblemParam {
  bool is_2d{false}; // "true" if 2d scenarios and "false" if 3d scenarios
};
struct ObstacleParam {
  float planning_horizon{1.0f};
};

} // namespace los_keeper
struct ObjectState {
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float rx;
  float ry;
  float rz;
};
struct DroneState {
  float px;
  float py;
  float pz;
  float vx;
  float vy;
  float vz;
  float ax;
  float ay;
  float az;
};

struct Point {
  float x{0.0};
  float y{0.0};
  float z{0.0};
};

struct StatePoly {
  BernsteinPoly px;
  BernsteinPoly py;
  BernsteinPoly pz;
  float rx;
  float ry;
  float rz;

  void SetDegree(const int &degree) {
    px.SetDegree(degree), py.SetDegree(degree), pz.SetDegree(degree);
  };

  void SetTimeInterval(float time_interval[2]) {
    px.SetTimeInterval(time_interval), py.SetTimeInterval(time_interval),
        pz.SetTimeInterval(time_interval);
  };

  Point GetPointAtTime(double time) const { return Point(); };
};

using namespace std;
typedef vector<int> IndexList;
typedef vector<IndexList> IndexListSet;
typedef vector<Point> PointList;
typedef vector<PointList> PointListSet;
typedef vector<StatePoly> PrimitiveList;
typedef vector<PrimitiveList> PrimitiveListSet;
#endif // LOS_KEEPER_TYPE_MANAGER_H
