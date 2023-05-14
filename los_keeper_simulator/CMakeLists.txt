cmake_minimum_required(VERSION 3.5)
project(los_keeper_simulator)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(los_keeper_msgs REQUIRED)

add_library(los_keeper_simulator src/los_keeper_simulator/los_keeper_simulator.cc)
ament_target_dependencies(los_keeper_simulator rclcpp los_keeper_msgs)

target_include_directories(
        los_keeper_simulator PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)
#target_link_libraries(los_keeper_simulator los_keeper_msgs)
install(FILES include/${PROJECT_NAME}/los_keeper_simulator.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/los_keeper_simulator)

add_executable(los_keeper_simulator_node src/los_keeper_simulator/los_keeper_simulator_node.cc)
target_link_libraries(los_keeper_simulator_node los_keeper_simulator)
install(TARGETS los_keeper_simulator_node DESTINATION lib/${PROJECT_NAME})