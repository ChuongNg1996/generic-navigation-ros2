cmake_minimum_required(VERSION 3.5)
project(generic_nav_msgs_srv_action_1_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED) # Find the package that generates message code from msg/srv files

rosidl_generate_interfaces(${PROJECT_NAME}  
 "msgs/GenericPose.msg"
 "msgs/GenericPoint.msg"
 "msgs/GenericOrientation.msg"
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
