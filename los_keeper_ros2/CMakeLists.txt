cmake_minimum_required(VERSION 3.5)
project(los_keeper_ros2)

#set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(los_keeper REQUIRED)
find_package(los_keeper_msgs REQUIRED)

add_library(los_server src/los_server/los_server.cc)
ament_target_dependencies(los_server rclcpp los_keeper_msgs sensor_msgs
                          geometry_msgs)
target_include_directories(
  los_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)
target_link_libraries(los_server los_keeper)
install(FILES include/${PROJECT_NAME}/los_server/los_server.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/los_server)

add_library(visualizer src/visualization/visualizer.cc)
ament_target_dependencies(visualizer rclcpp sensor_msgs geometry_msgs
                          visualization_msgs)
target_include_directories(
  visualizer PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                    $<INSTALL_INTERFACE:include>)
target_link_libraries(visualizer los_keeper)
install(
  FILES include/${PROJECT_NAME}/visualization/visualizer.h
  DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${PROJECT_NAME}/visualization)

add_executable(los_server_node src/los_server/los_server_node.cc)
target_link_libraries(los_server_node los_server visualizer)
install(TARGETS los_server_node DESTINATION lib/${PROJECT_NAME})

enable_testing()
find_package(GTest REQUIRED)

add_executable(los_server_test test/los_server_test.cc)
target_link_libraries(los_server_test los_server GTest::GTest GTest::Main)
add_test(los_server_test los_server_test)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
