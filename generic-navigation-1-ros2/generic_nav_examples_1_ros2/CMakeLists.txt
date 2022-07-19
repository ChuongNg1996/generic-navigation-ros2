cmake_minimum_required(VERSION 3.5)
project(generic_nav_examples_1_ros2)

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
find_package(rclcpp REQUIRED)                               # C++ Wrapper
find_package(generic_nav_msgs_srv_action_1_ros2 REQUIRED)   # Message/Service/Action
find_package(generic_nav_map_1_ros2 REQUIRED)               # Map
find_package(generic_nav_global_planner_1_ros2 REQUIRED)    # Global Path Planner
find_package(generic_nav_controller_1_ros2 REQUIRED)        # Controller
find_package(generic_nav_localization_1_ros2 REQUIRED)      # Localization
find_package(tf2_ros REQUIRED)                              # tf2
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(gazebo_ros REQUIRED)

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

# --------------------------------------------------- #
# ------------------ FOLDER PATHS ------------------- #
# --------------------------------------------------- #

# ROS Launch folder
install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

# urdf folder
install(
    DIRECTORY urdf
    DESTINATION share/${PROJECT_NAME}
)

# maps folder
# install(
#     DIRECTORY maps
#     DESTINATION share/${PROJECT_NAME}
# )

# worlds folder
install(
    DIRECTORY worlds
    DESTINATION share/${PROJECT_NAME}
)

# rviz folder
# install(
#     DIRECTORY rviz
#     DESTINATION share/${PROJECT_NAME}
# )

# params folder
# install(
#   DIRECTORY params
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
