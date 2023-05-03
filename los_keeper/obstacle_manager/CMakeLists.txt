find_package(PCL REQUIRED COMPONENTS common io)
find_package(Eigen REQUIRED)

add_library(obstacle_manager ObstacleManager.cpp)
# Link_libraries is enough to include all the pcl headers
target_link_libraries(obstacle_manager PUBLIC ${PCL_LIBRARIES})

# If another target links obstacle_manager, enable it to use all the includes of
# obstacle_manager by PUBLIC keyward. I used BUILD_INTERFACE and
# INSTALL_INTERFACE as the library's include path is different
target_include_directories(
  obstacle_manager PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
                          $<INSTALL_INTERFACE:include> ${EIGEN_INCLUDE_DIR})

# This will add obstacle manager to export files
install(
  TARGETS obstacle_manager
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)
install(FILES ObstacleManager.hpp DESTINATION include/obstacle_manager)

# Add testing. GTest::GTest and GTest::Main should be linked, assuming that
# parent cmakelist aleardy found gTest
add_executable(obstacle_manager_test obstacle_manager_test.cpp)
target_link_libraries(obstacle_manager_test obstacle_manager GTest::GTest
                      GTest::Main)
add_test(obstacle_manager_test obstacle_manager_test)

# For debugging: get_property( INC_DIR_LIST TARGET obstacle_manager PROPERTY
# INTERFACE_INCLUDE_DIRECTORIES) message("obstacle manager includes:
# ${INC_DIR_LIST}")
