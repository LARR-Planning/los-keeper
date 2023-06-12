//
// Created by larr-planning on 23. 5. 16.
//
#ifndef HEADER_TYPE_MANAGER
#define HEADER_TYPE_MANAGER
#include "los_keeper/bernstein_polynomial_utils/bernstein_polynomial_utils.h"
#include "thread"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace los_keeper {
typedef pcl::PointXYZ PclPoint;
typedef pcl::PointCloud<PclPoint> PclPointCloud;

struct PlanningParameter {
  double replan_period_sec{1.0};
  struct {
    int num_sample{0};
    int num_thread{0};
    bool is_lite{false};
  } sampling;
  struct {
    float planning{0.0f};
  } horizon;
  struct {
    float obstacle_max{0.0f};
    float target_min{0.0f};
    float target_max{0.0f};
  } distance;
  struct {
    float vel_max{0.0f};
    float acc_max{0.0f};
  } dynamic_limits;
  struct {
    float rx{0.0f};
    float ry{0.0f};
    float rz{0.0f};
  } drone;
  struct {
    float height{0.0f};
    float width{0.0f};
  } virtual_pcl_bbox;
};

struct PredictionParameter {
  struct {
    int num_sample{0};
    int num_thread{0};
    bool is_lite{false};
  } sampling;
  struct {
    float prediction{0.0f};
  } horizon;
  struct {
    float vel_max{0.0f};
    float acc_max{0.0f};
  } dynamic_limits;
  struct {
    float obstacle_max{0.0f};
  } distance;
  struct {
    float height{0.0f};
    float width{0.0f};
  } virtual_pcl_bbox;
};
struct ObstacleParameter {
  float planning_horizon{0.0f};
};

struct ProblemParameter {
  bool is_2d{false};
};

struct Parameters {
  ProblemParameter problem;
  PlanningParameter planning;
  PredictionParameter prediction;
  ObstacleParameter obstacle;
};

} // namespace los_keeper
struct ObjectState {
  double t_sec{0.0};
  size_t id{0};
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
  double t_sec{0.0};
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

struct JerkControlInput {
  double t_sec{0.0};
  float jx{0.0f};
  float jy{0.0f};
  float jz{0.0f};
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
#endif /* HEADER_TYPE_MANAGER */
