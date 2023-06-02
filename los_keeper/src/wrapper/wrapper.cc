#include "los_keeper/wrapper/wrapper.h"

using namespace los_keeper;

std::optional<Point> PlanningResult::GetPointAtTime(double t) const {
    if (!chasing_trajectory)
        return std::nullopt;
    return Point();
}

bool Wrapper::UpdateState(store::State &state) {
    bool is_changed = false;
    // state.is_initialized =
    //     obstacle_manager_->IsInitialized() && target_manager_->IsInitialized();
    // state.is_currently_safe =
    //     obstacle_manager_->CheckCollisionOnPosition(drone_state);
    // state.is_planning_safe =
    //     obstacle_manager_->CheckCollisionAlongTrajectory(drone_state);
    return is_changed;
}

void Wrapper::HandleStopAction() { state_.is_activated = false; }

void Wrapper::HandleActivateAction() { state_.is_activated = true; }

void Wrapper::HandleReplanAction() {
    PlanningProblem planning_problem;
    {
        std::scoped_lock lock(mutex_list_.drone_state, mutex_list_.point_cloud);
        planning_problem.drone_state = drone_state_;
        planning_problem.point_cloud = obstacle_manager_->GetPointCloud();
        // TODO(@): add target_state_list and structure_obstacle_poly_list
    }
    const auto &point_cloud = planning_problem.point_cloud;
    const auto &structured_obstacle_poly_list = planning_problem.structured_obstacle_poly_list;

    PlanningResult new_planning_result;
    auto target_prediction_list = target_manager_->PredictTargetList(
            planning_problem.target_state_list, point_cloud, structured_obstacle_poly_list);
    if (!target_prediction_list)
        goto update;

    new_planning_result.chasing_trajectory = trajectory_planner_->ComputeChasingTrajectory(
            target_prediction_list.value(), point_cloud, structured_obstacle_poly_list);

    update :
    {
        std::unique_lock<std::mutex> lock(mutex_list_.control);
        planning_result_ = new_planning_result;
    }
}

Wrapper::Wrapper() {
    // TODO: Parameter Assignment? (los_server <-> wrapper <-> (prediction and planning module))
    /*
     * If ProblemParam.is_2d = true?
     * target_manager_.reset(new TargetManager2D(prediction_param));
     * trajectory_planner_.reset(new TrajectoryPlanner2D(planning_param));
     * If ProblemParam.is_2d = false?
     *  target_manager_.reset(new TargetManager3D(prediction_param));
     *  target_manager_.reset(new TrajectoryPlanner3D(planning_param));
     */
    target_manager_.reset(new TargetManager3D);
}

void Wrapper::OnPlanningTimerCallback() {

    UpdateState(state_);
    auto action = DecideAction(state_);
    switch (action) {
        case store::Action::kInitialize:
            HandleActivateAction();
        case store::Action::kStop:
            HandleStopAction();
        default:
            HandleReplanAction();
    }
}

void Wrapper::OnStartServiceCallback() {
    using namespace store;
    HandleActivateAction();
}

void Wrapper::SetPoints(const pcl::PointCloud<pcl::PointXYZ> &points) {
    std::unique_lock<std::mutex> lock(mutex_list_.point_cloud, std::defer_lock);
    if (lock.try_lock()) {
        obstacle_manager_->SetObstacleCloud(points);
    }
}

void Wrapper::SetDroneState(const DroneState &drone_state) {
    std::unique_lock<std::mutex> lock(mutex_list_.drone_state, std::defer_lock);
    if (lock.try_lock()) {
        // TODO(@): set whoever need this
        drone_state_ = drone_state;
    }
}

std::optional<Point> Wrapper::GenerateControlInputFromPlanning(double time) {
    // TODO(@): generate jerk
    std::optional<Point> control_input;
    {
        std::unique_lock<std::mutex> lock(mutex_list_.control);
        control_input = planning_result_.GetPointAtTime(time);
    }
    return control_input;
}
