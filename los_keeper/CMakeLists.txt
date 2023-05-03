cmake_minimum_required(VERSION 3.5)
project(los_keeper)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIRS})

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
function(add_los_library library_folder dependency_list)
  message("found library: ${library_folder}")
  list(APPEND ${LIBRARY_LIST} ${library_folder})
  file(GLOB LIBRARY_SOURCES
       "${CMAKE_CURRENT_SOURCE_DIR}/${library_folder}/*.cpp")
  file(GLOB LIBRARY_HEADERS
       "${CMAKE_CURRENT_SOURCE_DIR}/${library_folder}/*.hpp")
  add_library(${library_folder} SHARED ${LIBRARY_SOURCES} ${LIBRARY_HEADERS})
  target_link_libraries(${library_folder} ${dependency_list})
  install(FILES ${LIBRARY_HEADERS}
          DESTINATION include/los_keeper/${library_folder})

endfunction()

add_los_library(obstacle_manager "${PCL_COMMON}")
add_los_library(target_manager "")

export(TARGETS obstacle_manager target_manager
       FILE "${CMAKE_CURRENT_BINARY_DIR}/los_keeperConfig.cmake")

install(
  TARGETS obstacle_manager target_manager
  EXPORT los_keeper
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES
  DESTINATION include)
install(
  EXPORT los_keeper
  FILE los_keeperConfig.cmake
  NAMESPACE los_keeper::
  DESTINATION lib/cmake/los_keeper)
