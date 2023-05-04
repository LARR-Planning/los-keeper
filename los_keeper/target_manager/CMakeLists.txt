add_library(target_manager target_manager.cc)
# as obstacle_manager brings all the necessary headers (PUBLIC
# target_include_directories), I do not have to target_include_directories here
target_link_libraries(target_manager PUBLIC obstacle_manager)

install(
  TARGETS target_manager
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)

install(FILES target_manager.h DESTINATION include/los_keeper/target_manager)

add_executable(target_manager_test target_manager_test.cc)
target_link_libraries(target_manager_test target_manager GTest::GTest
                      GTest::Main)
add_test(target_manager_test target_manager_test)
