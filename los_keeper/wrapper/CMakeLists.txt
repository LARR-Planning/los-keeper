add_library(wrapper Wrapper.cpp)
target_link_libraries(wrapper PUBLIC target_manager)

install(
  TARGETS wrapper
  EXPORT ${EXPORT_NAME}
  LIBRARY DESTINATION lib)

install(FILES Wrapper.hpp DESTINATION include/wrapper)

add_executable(wrapper_test wrapper_test.cpp)
target_link_libraries(wrapper_test wrapper GTest::GTest GTest::Main)
add_test(wrapper_test wrapper_test)
