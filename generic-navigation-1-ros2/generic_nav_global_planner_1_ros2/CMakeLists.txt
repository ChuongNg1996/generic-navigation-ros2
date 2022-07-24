cmake_minimum_required(VERSION 3.5)
project(generic_nav_global_planner_1_ros2)

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

# Find the packages
# https://cmake.org/cmake/help/latest/command/find_package.html
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(grid_dijkstra_8dir_1 src/grid_dijkstra_8dir_1.cpp)
ament_target_dependencies(grid_dijkstra_8dir_1 rclcpp std_msgs geometry_msgs tf2_geometry_msgs)

add_executable(custom_trajectory src/custom_trajectory.cpp)
ament_target_dependencies(custom_trajectory rclcpp std_msgs geometry_msgs tf2_geometry_msgs)

# add the install(TARGETS…) section so ros2 run can FIND your executable
install(TARGETS
  grid_dijkstra_8dir_1
  custom_trajectory
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
