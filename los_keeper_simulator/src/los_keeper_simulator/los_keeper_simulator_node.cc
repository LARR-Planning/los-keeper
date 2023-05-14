
#include "los_keeper_simulator/los_keeper_simulator.h"
int main(int argc, char **argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<LosKeeperSimulator>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
