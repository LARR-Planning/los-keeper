add_library(wrapper wrapper.cc)
target_link_libraries(wrapper PUBLIC target_manager)

install(
  TARGETS wrapper
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)

install(FILES wrapper.h DESTINATION include/los_keeper/wrapper)

add_executable(wrapper_test wrapper_test.cc)
target_link_libraries(wrapper_test wrapper GTest::GTest GTest::Main)
add_test(wrapper_test wrapper_test)
