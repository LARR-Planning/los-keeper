cmake_minimum_required(VERSION 3.5)
project(los_keeper_msgs)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files "msg/ChaserState.msg" "msg/ObjectState.msg"
        "msg/ObjectStateArray.msg" "msg/JerkControlInput.msg"
        "msg/VelocityControlInput.msg" "msg/AccelControlInput.msg")

set(srv_files "srv/ToggleActivate.srv")

rosidl_generate_interfaces(${PROJECT_NAME} ${msg_files} ${srv_files}
        DEPENDENCIES std_msgs)

ament_package()
