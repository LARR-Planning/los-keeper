cmake_minimum_required(VERSION 3.5)
project(los_keeper_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(los_keeper REQUIRED)

enable_testing()
find_package(GTest REQUIRED)

add_library(los_server los_server/los_server.cc)
ament_target_dependencies(los_server rclcpp)
target_include_directories(
  los_server PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)
target_link_libraries(los_server los_keeper::wrapper)
install(FILES los_server/los_server.h
        DESTINATION ${CMAKE_INSTALL_PREFIX}/include/los_server)

add_executable(los_server_test los_server/los_server_test.cc)
target_link_libraries(los_server_test los_server GTest::GTest GTest::Main)
add_test(los_server_test los_server_test)

ament_package()
