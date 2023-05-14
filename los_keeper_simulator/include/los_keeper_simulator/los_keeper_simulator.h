//
// Created by larr-planning on 23. 5. 14.
//

#ifndef LOS_KEEPER_LOS_KEEPER_SIMULATOR_H
#define LOS_KEEPER_LOS_KEEPER_SIMULATOR_H


#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <fstream>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "los_keeper_simulator/los_keeper_simulator.h"
#include "los_keeper_simulator/los_keeper_simulator_math_utils.h"
#include "los_keeper_simulator/los_keeper_simulator_utils.h"

#include "los_keeper_msgs/msg/object_state.hpp"
#include "los_keeper_msgs/msg/object_state_array.hpp"
#include "los_keeper_msgs/msg/drone_state.hpp"
#include "los_keeper_msgs/msg/jerk_control_input.hpp"

using std::placeholders::_1;

class LosKeeperSimulator: public rclcpp::Node{
private:
    std::chrono::system_clock::time_point t0_;
    rclcpp::TimerBase::SharedPtr timer_;

    State keeper_state_;
    vector<State> current_obstacle_list_;
    State current_target_;
    vector<ObjectState> object_list_;
    ControlInput keeper_ctrl_input_;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;

    los_keeper_msgs::msg::DroneState current_drone_state_;
    los_keeper_msgs::msg::ObjectState current_target_state_;
    los_keeper_msgs::msg::ObjectStateArray current_obstacle_array_state_;
    
    /*
     * Variables for publishing msg (topic)
     */
    visualization_msgs::msg::MarkerArray obstacle_list_vis_;
    visualization_msgs::msg::Marker target_vis_;
    visualization_msgs::msg::Marker drone_vis_;
    visualization_msgs::msg::Marker bearing_vector_vis_;

    /*
     * ROS Publisher
     */
    // Visualization
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_obstacle_list_vis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_target_vis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_drone_vis_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_bearing_vector_vis_;
    // Control Msgs
    rclcpp::Publisher<los_keeper_msgs::msg::ObjectState>::SharedPtr pub_target_state_;
    rclcpp::Publisher<los_keeper_msgs::msg::ObjectStateArray>::SharedPtr pub_obstacle_list_state_;
    rclcpp::Publisher<los_keeper_msgs::msg::DroneState>::SharedPtr pub_drone_state_;

    /*
     *  ROS Subscriber
     */
    rclcpp::Subscription<los_keeper_msgs::msg::JerkControlInput>::SharedPtr sub_jerk_control_input_;
    void CallbackJerkControlInput(const los_keeper_msgs::msg::JerkControlInput::SharedPtr msg);

    std::string map_frame_id_;
    bool is_2d_simulation_;
    double simulation_dt_;
    double object_size_;
    int num_object_;
    int target_id_;
    int num_obstacle_;
    std::string initial_state_file_name_;
    std::string object_trajectory_file_name_;

    /*
     * ROS Subscriber
     */
    void PrepareRosMsgs(float t);
    void PublishRosMsgs(float t);
    void UpdateDynamics(float t);
    float GetCurrentTime();
    void ReadInitialState();
    void ReadObjectTrajectory();
    void UpdateParameters();
    void RunSimulation();
    void RunWhileLoop();
public:
    LosKeeperSimulator();

};



#endif //LOS_KEEPER_LOS_KEEPER_SIMULATOR_H
