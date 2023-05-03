cmake_minimum_required(VERSION 3.5)
project(los_keeper)

enable_testing()
find_package(GTest REQUIRED)

# We will append contents of this config file along subdirectory
set(EXPORT_NAME ${PROJECT_NAME}Config)

# TODO: utils and chasing planner should be added
add_subdirectory(obstacle_manager)
add_subdirectory(target_manager)
add_subdirectory(wrapper)

install(
  EXPORT ${EXPORT_NAME}
  NAMESPACE los_keeper::
  DESTINATION share/${PROJECT_NAME}/cmake)
