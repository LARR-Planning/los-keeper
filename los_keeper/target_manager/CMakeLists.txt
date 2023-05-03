add_library(target_manager TargetManager.cpp)
# as obstacle_manager brings all the necessary headers (PUBLIC
# target_include_directories), I do not have to target_include_directories here
target_link_libraries(target_manager PUBLIC obstacle_manager)

install(
  TARGETS target_manager
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)

install(FILES TargetManager.hpp DESTINATION include/target_manager)

add_executable(target_manager_test target_manager_test.cpp)
target_link_libraries(target_manager_test target_manager GTest::GTest
                      GTest::Main)
add_test(target_manager_test target_manager_test)
