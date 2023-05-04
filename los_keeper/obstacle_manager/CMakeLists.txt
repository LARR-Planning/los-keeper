find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen REQUIRED)

add_library(obstacle_manager obstacle_manager.cc)
target_link_libraries(obstacle_manager PUBLIC ${PCL_LIBRARIES})

target_include_directories(
  obstacle_manager
  PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
         $<INSTALL_INTERFACE:include/los_keeper> $<INSTALL_INTERFACE:include>
         ${EIGEN_INCLUDE_DIR} ${PCL_INCLUDE_DIRS})

install(
  TARGETS obstacle_manager
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)
install(FILES obstacle_manager.h
        DESTINATION include/los_keeper/obstacle_manager)

add_executable(obstacle_manager_test obstacle_manager_test.cc)
target_link_libraries(obstacle_manager_test obstacle_manager GTest::GTest
                      GTest::Main)
add_test(obstacle_manager_test obstacle_manager_test)

# For debugging: get_property( INC_DIR_LIST TARGET obstacle_manager PROPERTY
# INTERFACE_INCLUDE_DIRECTORIES) message("obstacle manager includes:
# ${INC_DIR_LIST}")
