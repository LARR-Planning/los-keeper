#include "los_keeper_simulator/los_keeper_simulator.h" 

void LosKeeperSimulator::RunSimulation() {
    using namespace std::chrono_literals;
    ReadInitialState();
    ReadObjectTrajectory();
    timer_  = this->create_wall_timer(5ms,std::bind(&LosKeeperSimulator::RunWhileLoop,this));
}

LosKeeperSimulator::LosKeeperSimulator(): Node("los_keeper_simulator_node"){
    t0_ = std::chrono::system_clock::now();
    pub_obstacle_list_vis_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("obstacle_list_vis",1);
    pub_drone_vis_ = this->create_publisher<visualization_msgs::msg::Marker>("drone_vis",1);
    pub_bearing_vector_vis_ = this->create_publisher<visualization_msgs::msg::Marker>("bearing_vis",1);
    pub_target_vis_ = this->create_publisher<visualization_msgs::msg::Marker>("target_vis",1);

    pub_target_state_ = this->create_publisher<los_keeper_msgs::msg::ObjectState>("target_state",1);
    pub_obstacle_list_state_ = this->create_publisher<los_keeper_msgs::msg::ObjectStateArray>("obstacle_list_state",1);
    pub_drone_state_ = this->create_publisher<los_keeper_msgs::msg::DroneState>("drone_state",1);
    sub_jerk_control_input_ = this->create_subscription<los_keeper_msgs::msg::JerkControlInput>("jerk_control_input",1,
                                                                                               std::bind(&LosKeeperSimulator::CallbackJerkControlInput, this, _1));
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

    UpdateParameters();
    std::cout<<"IS 2D?: "<<is_2d_simulation_<<std::endl;
    RunSimulation();
}

void LosKeeperSimulator::RunWhileLoop() {
    float cur_time = GetCurrentTime();
    UpdateDynamics(cur_time);
    PrepareRosMsgs(cur_time);
    PublishRosMsgs(cur_time);
}

void LosKeeperSimulator::UpdateParameters() {
    this->declare_parameter("map_frame_id","");
    map_frame_id_= get_parameter("map_frame_id").get_value<std::string>();
    this->declare_parameter("is_2d_simulation",true);
    is_2d_simulation_ = get_parameter("is_2d_simulation").get_value<bool>();
    this->declare_parameter("simulation_dt",0.01);
    simulation_dt_ = get_parameter("simulation_dt").get_value<double>();
    this->declare_parameter("object_size",0.001);
    object_size_ = get_parameter("object_size").get_value<double>();
    this->declare_parameter("num_object",1);
    num_object_ = (int)get_parameter("num_object").get_value<int>();
    this->declare_parameter("target_id",0);
    target_id_ = (int) get_parameter("target_id").get_value<int>();
    num_obstacle_ = num_object_ - 1;
    this->declare_parameter("initial_state_file_name","");
    initial_state_file_name_ = get_parameter("initial_state_file_name").get_value<std::string>();
    this->declare_parameter("object_trajectory_file_name","");
    object_trajectory_file_name_ = get_parameter("object_trajectory_file_name").get_value<std::string>();
}

void LosKeeperSimulator::PrepareRosMsgs(float t) {
    {   // OBSTACLE INFORMATION;
        obstacle_list_vis_.markers.clear();
        current_obstacle_array_state_.object_state_array.clear();
        visualization_msgs::msg::Marker tempMarker;
        tempMarker.header.frame_id= map_frame_id_;
        tempMarker.color.a = 0.5;
        tempMarker.color.r = 0.5;
        tempMarker.color.g = 0.5;
        tempMarker.color.b = 0.5;
        if(is_2d_simulation_)
            tempMarker.type = visualization_msgs::msg::Marker::CYLINDER;
        else
            tempMarker.type = visualization_msgs::msg::Marker::SPHERE;
        for(int i =0;i<num_obstacle_;i++){
            tempMarker.pose.position.x = current_obstacle_list_[i].px;
            tempMarker.pose.position.y = current_obstacle_list_[i].py;
            tempMarker.pose.position.z = current_obstacle_list_[i].pz;
            tempMarker.pose.orientation.w = 1.0;
            tempMarker.pose.orientation.x = 0.0;
            tempMarker.pose.orientation.y = 0.0;
            tempMarker.pose.orientation.z = 0.0;
            tempMarker.scale.x = 2*object_size_;
            tempMarker.scale.y = 2*object_size_;
            if(is_2d_simulation_)
                tempMarker.scale.z = 2.0;
            else
                tempMarker.scale.z = 2*object_size_;
            tempMarker.id = i;
            tempMarker.ns = to_string(i);
            obstacle_list_vis_.markers.push_back(tempMarker);
            // Customized Msg
            los_keeper_msgs::msg::ObjectState temp_status;
            temp_status.px = current_obstacle_list_[i].px;
            temp_status.py = current_obstacle_list_[i].py;
            temp_status.pz = current_obstacle_list_[i].pz;
            temp_status.vx = current_obstacle_list_[i].vx;
            temp_status.vy = current_obstacle_list_[i].vy;
            temp_status.vz = current_obstacle_list_[i].vz;
            current_obstacle_array_state_.object_state_array.push_back(temp_status);
        }
    }
    {   //TARGET INFORMATION
        if(not object_list_.empty()){
            target_vis_.header.frame_id = map_frame_id_;
            target_vis_.color.a = 0.5;
            target_vis_.color.r = 1.0;
            target_vis_.color.g = 0.0;
            target_vis_.color.b = 0.0;
            if(is_2d_simulation_)
                target_vis_.type = visualization_msgs::msg::Marker::CYLINDER;
            else
                target_vis_.type = visualization_msgs::msg::Marker::SPHERE;
            target_vis_.pose.position.x = current_target_.px;
            target_vis_.pose.position.y = current_target_.py;
            target_vis_.pose.position.z = current_target_.pz;
            target_vis_.pose.orientation.w = 1.0;
            target_vis_.pose.orientation.x = 0.0;
            target_vis_.pose.orientation.y = 0.0;
            target_vis_.pose.orientation.z = 0.0;
            target_vis_.scale.x= 2*object_size_;
            target_vis_.scale.y= 2*object_size_;
            if(is_2d_simulation_)
                target_vis_.scale.z= 2.0;
            else
                target_vis_.scale.z= 2*object_size_;
            current_target_state_.px = current_target_.px;
            current_target_state_.py = current_target_.py;
            current_target_state_.pz = current_target_.pz;
            current_target_state_.vx = current_target_.vx;
            current_target_state_.vy = current_target_.vy;
            current_target_state_.vz = current_target_.vz;
        }
    }
    {   // DRONE INFORMATION
        drone_vis_.header.frame_id = map_frame_id_;
        if(is_2d_simulation_)
            drone_vis_.type = visualization_msgs::msg::Marker::CYLINDER;
        else
            drone_vis_.type = visualization_msgs::msg::Marker::SPHERE;
        drone_vis_.color.a = 0.5;
        drone_vis_.color.r = 0.0;
        drone_vis_.color.g = 0.0;
        drone_vis_.color.b = 1.0;
        drone_vis_.pose.position.x = keeper_state_.px;
        drone_vis_.pose.position.y = keeper_state_.py;
        drone_vis_.pose.position.z = keeper_state_.pz;
        drone_vis_.pose.orientation.w = 1.0;
        drone_vis_.pose.orientation.x = 0.0;
        drone_vis_.pose.orientation.y = 0.0;
        drone_vis_.pose.orientation.z = 0.0;
        drone_vis_.scale.x = 2*object_size_;
        drone_vis_.scale.y = 2*object_size_;
        if(is_2d_simulation_)
            drone_vis_.scale.z = 2.0;
        else
            drone_vis_.scale.z = 2*object_size_;
        current_drone_state_.px = keeper_state_.px;
        current_drone_state_.py = keeper_state_.py;
        current_drone_state_.pz = keeper_state_.pz;
        current_drone_state_.vx = keeper_state_.vx;
        current_drone_state_.vy = keeper_state_.vy;
        current_drone_state_.vz = keeper_state_.vz;
        current_drone_state_.ax = keeper_state_.ax;
        current_drone_state_.ay = keeper_state_.ay;
        current_drone_state_.az = keeper_state_.az;
    }

    {   //Bearing Vector Visualization
        bearing_vector_vis_.header.frame_id = map_frame_id_;
        bearing_vector_vis_.type = visualization_msgs::msg::Marker::ARROW;
        bearing_vector_vis_.color.a = 0.7;
        bearing_vector_vis_.color.r = 0.1;
        bearing_vector_vis_.color.g = 1.0;
        bearing_vector_vis_.color.b = 0.0;
        bearing_vector_vis_.points.clear();
        geometry_msgs::msg::Point start_point;
        start_point.x = keeper_state_.px, start_point.y = keeper_state_.py, start_point.z = keeper_state_.pz;
        geometry_msgs::msg::Point end_point;
        end_point.x = current_target_.px, end_point.y = current_target_.py, end_point.z = current_target_.pz;
        bearing_vector_vis_.points.push_back(start_point);
        bearing_vector_vis_.points.push_back(end_point);
        bearing_vector_vis_.scale.x = 0.025;
        bearing_vector_vis_.scale.y = 0.05;
    }
    {
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = this->get_clock()->now();
        transform_stamped.header.frame_id = map_frame_id_;
        transform_stamped.child_frame_id="keeper";
        transform_stamped.transform.translation.x = keeper_state_.px;
        transform_stamped.transform.translation.y = keeper_state_.py;
        transform_stamped.transform.translation.z = keeper_state_.pz;
        tf2::Quaternion q;
        q.setRPY(0,0,0);
        transform_stamped.transform.rotation.x = q.x();
        transform_stamped.transform.rotation.y = q.y();
        transform_stamped.transform.rotation.z = q.z();
        transform_stamped.transform.rotation.w = q.w();
        tf_static_broadcaster_->sendTransform(transform_stamped);
    }

}

void LosKeeperSimulator::PublishRosMsgs(float t) {
    {
        if (not obstacle_list_vis_.markers.empty())
            pub_obstacle_list_vis_->publish(obstacle_list_vis_);
    }
    {
        if(not obstacle_list_vis_.markers.empty()){
            pub_target_vis_->publish(target_vis_);
        }
    }
    {
        pub_drone_vis_->publish(drone_vis_);
    }
    {
        pub_target_state_->publish(current_target_state_);
        pub_drone_state_->publish(current_drone_state_);
        pub_obstacle_list_state_->publish(current_obstacle_array_state_);
        pub_bearing_vector_vis_->publish(bearing_vector_vis_);
    }
}

void LosKeeperSimulator::ReadInitialState() {
    std::ifstream initial_state_file;
    initial_state_file.open(initial_state_file_name_.c_str());
    if(initial_state_file.is_open()){
        initial_state_file>>keeper_state_.px>>keeper_state_.py>>keeper_state_.pz;
        initial_state_file>>keeper_state_.vx>>keeper_state_.vy>>keeper_state_.vz;
        initial_state_file>>keeper_state_.ax>>keeper_state_.ay>>keeper_state_.az;
    }
    else
        cout<<"UNABLE TO READ INITIAL STATE FILE NAME"<<endl;
}

void LosKeeperSimulator::ReadObjectTrajectory() {
    ifstream object_trajectory_file;
    object_trajectory_file.open(object_trajectory_file_name_.c_str());
    object_list_.clear();
    int num_read_unit = 12;
    string line, word;
    vector<string> row;
    vector<vector<string>> content;
    if(object_trajectory_file.is_open()){
        while(getline(object_trajectory_file, line)){
            row.clear();
            stringstream str(line);
//            int num_object_id = 0;
            while(getline(str,word,','))
                row.push_back(word);
            content.push_back(row);
        }
    }
    else{
        cout<<"UNABLE TO READ OBJECT TRAJECTORY HISTORY"<<endl;
    }
    ObjectState temp_obstacle;
    for(int i = 0;i<num_object_;i++)
        object_list_.push_back(temp_obstacle);
    for(int i = 0;i<num_object_;i++){
        for(int j = 0;j<(int)content.size();j++){
            object_list_[i].t.push_back(stod(content[j][i*num_read_unit+1]));
            object_list_[i].px.push_back(stod(content[j][i*num_read_unit+2]));
            object_list_[i].py.push_back(stod(content[j][i*num_read_unit+3]));
            object_list_[i].pz.push_back(stod(content[j][i*num_read_unit+4]));
            object_list_[i].vx.push_back(stod(content[j][i*num_read_unit+5]));
            object_list_[i].vy.push_back(stod(content[j][i*num_read_unit+6]));
            object_list_[i].vz.push_back(stod(content[j][i*num_read_unit+7]));
        }
    }
//    std::cout<<"The number of objects: "<<object_list_.size()<<std::endl;
//    std::cout<<"PARAM: The number of objects: "<<num_object_<<std::endl;
//    std::cout<<"HISTORY SIZE: "<<object_list_[0].t.size()<<std::endl;

}

float LosKeeperSimulator::GetCurrentTime() {
    std::chrono::duration<float> sec_float = std::chrono::system_clock::now() - t0_;
    return sec_float.count();
}

void LosKeeperSimulator::UpdateDynamics(float t) {
    current_obstacle_list_.clear();
    State temp_state;
    for(int i =0;i<num_object_;i++){
        if(i != target_id_){
            temp_state.px = interpolate(object_list_[i].t,object_list_[i].px,t);
            temp_state.py = interpolate(object_list_[i].t,object_list_[i].py,t);
            temp_state.pz = interpolate(object_list_[i].t,object_list_[i].pz,t);
            temp_state.vx = interpolate(object_list_[i].t,object_list_[i].vx,t);
            temp_state.vy = interpolate(object_list_[i].t,object_list_[i].vy,t);
            temp_state.vz = interpolate(object_list_[i].t,object_list_[i].vz,t);
            current_obstacle_list_.push_back(temp_state);
        }
        else{
            current_target_.px = interpolate(object_list_[i].t,object_list_[i].px,t);
            current_target_.py = interpolate(object_list_[i].t,object_list_[i].py,t);
            current_target_.pz = interpolate(object_list_[i].t,object_list_[i].pz,t);
            current_target_.vx = interpolate(object_list_[i].t,object_list_[i].vx,t);
            current_target_.vy = interpolate(object_list_[i].t,object_list_[i].vy,t);
            current_target_.vz = interpolate(object_list_[i].t,object_list_[i].vz,t);
        }
    }
    static float t_prev = t;
    float dt = t - t_prev;
    keeper_state_.px = keeper_state_.px + keeper_state_.vx * dt;
    keeper_state_.py = keeper_state_.py + keeper_state_.vy * dt;
    keeper_state_.pz = keeper_state_.pz + keeper_state_.vz * dt;
    keeper_state_.vx = keeper_state_.vx + keeper_state_.ax * dt;
    keeper_state_.vy = keeper_state_.vy + keeper_state_.ay * dt;
    keeper_state_.vz = keeper_state_.vz + keeper_state_.az * dt;
    keeper_state_.ax = keeper_state_.ax + keeper_ctrl_input_.jx * dt;
    keeper_state_.ay = keeper_state_.ay + keeper_ctrl_input_.jy * dt;
    keeper_state_.az = keeper_state_.az + keeper_ctrl_input_.jz * dt;
//    cout<<"Keeper x: "<<current_target.px<<" y: "<<current_target.py<<" z: "<<current_target.pz<<endl;
    t_prev = t;

}

void LosKeeperSimulator::CallbackJerkControlInput(const los_keeper_msgs::msg::JerkControlInput::SharedPtr msg) {
    keeper_ctrl_input_.jx = msg->jx;
    keeper_ctrl_input_.jy = msg->jy;
    keeper_ctrl_input_.jz = msg->jz;
}
