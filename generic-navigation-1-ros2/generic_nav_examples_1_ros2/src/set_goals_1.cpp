#include <iostream>                                     // IO operations
#include <chrono>                                       // For time utilities


#include "rclcpp/rclcpp.hpp"                            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;                   // For time utilities

    class global_planner_node : public rclcpp::Node
    {
    };
