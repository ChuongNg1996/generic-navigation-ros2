#include <iostream>                                     // IO operations
#include <vector>                                       // Vector container
#include <chrono>                                       // For time utilities
#include <fstream>                                      // Input file
#include <stdio.h>                                      // Standard IO operations
#include <string.h>                                     // String operations

#include "rclcpp/rclcpp.hpp"                            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/twist.hpp"                  // Always hpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;                   // For time utilities

    class global_planner_node : public rclcpp::Node
    {
        private:
            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //

            std_msgs::msg::Bool in_process_init;        // Indicate whether lower controller is in process or not
            typedef geometry_msgs::msg::PoseStamped GoalType;   // Define common type for sub goals
            GoalType sub_goal;                          // a goal unit
            std::vector<GoalType> sub_goals;            // vector of sub goals
            std::ifstream indata;                       // File reader     
            int i = 0;                                  // Index of list of sub goals
            int max;                                    // Number of sub goals
            
            // ---------------------------------------------------------- //
            // ---------            ROS Variables               --------- //
            // ---------------------------------------------------------- //
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_process_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pub_;
            rclcpp::TimerBase::SharedPtr sub_goal_timer_;

            // ---------------------------------------------------------- //
            // ---------          Callback Funtions             --------- //
            // ---------------------------------------------------------- //

            // Receive from lower controller that no process is running - ready to send NEW SUB GOAL
            void ctrl_process_callback (const std_msgs::msg::Bool::SharedPtr msg)
            {
                in_process_init.data = msg->data;
            }

            // Check whether lower controller is free or not to send NEW SUB GOAL
            void sub_goal_callback()
            {
                // If in_process_init = FALSE, lower controller is NOT in a process
                if (!in_process_init.data) 
                {
                    // If there are sub goals left
                    if (i < max)
                    {
                        // Then send the sub goals
                        sub_goal_pub_->publish(sub_goals[i]);
                        //std::cout << i << std::endl;
                        i++;
                        // AUTO set the state of process = TRUE to prevent sending new goals
                        // Waiting for lower controller to set in_process_init = TRUE is not reliable
                        in_process_init.data = true;        
                    }
                }
            }

        public:
            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            global_planner_node() :  Node ("global_planner_node"), in_process_init(), sub_goal(), sub_goals(), indata(), max()
            {
                // Pub & Sub

                // Receive from lower controller that no process is running - ready to send NEW SUB GOAL
                ctrl_process_sub_ = this->create_subscription<std_msgs::msg::Bool>("ctrl_process_init", 10, std::bind(&global_planner_node::ctrl_process_callback, this, std::placeholders::_1));
                
                // Send NEW SUB GOAL
                sub_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10);

                in_process_init.data = true;

                //---------------------- READ FILES ----------------------// 
                
                // Read the file (Must be full path)
                indata.open("/home/chuong/ros2_ws/src/generic-navigation-ros2/generic-navigation-1-ros2/generic_nav_global_planner_1_ros2/custom_trajectories/trajectory_1.txt");

                // Check if there is any data/file has been correctly read
                if(!indata) { // file couldn't be opened
                    std::cout  << "Error: file could not be opened" << std::endl;
                    exit(1);
                }

                // This part depends on how data is arranged
                std::string s;                              // data in string format
                std::stringstream ss1;                      // data in string stream format
                ss1 << indata.rdbuf();                      // read the full data and put in string stream

                // separate data in '\n'
                while (getline(ss1, s, '\n')) {
                    std::stringstream ss2(s);
                    int j = 0;
                    // separate data in ','
                    while (getline(ss2, s, ','))
                    {
                        // Read individual value in string and convert to double
                        if (j == 0) sub_goal.pose.position.x = std::stod(s);    
                        else sub_goal.pose.position.y = std::stod(s);
                        j++;
                        //std::cout << s << std::endl;
                    }
                    max++;                                  // Increase number of data point
                    sub_goals.push_back(sub_goal);          // Store data in vector
                }
                //--------------------------------------------------------// 

                /* Algorithm TIMER */
                sub_goal_timer_ = this->create_wall_timer(
                10ms, std::bind(&global_planner_node::sub_goal_callback, this));

            }
    };



// ---------------------------------------------------------- //
// ---------                MAIN                    --------- //
// ---------------------------------------------------------- //

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<global_planner_node>());
    rclcpp::shutdown();
    return 0;
}
/*
+ Read whole ASCII file into C++ std::string:       https://www.tutorialspoint.com/Read-whole-ASCII-file-into-Cplusplus-std-string 

+ Split std::string:                                https://www.techiedelight.com/split-string-cpp-using-delimiter/
                                                    https://www.fluentcpp.com/2017/04/21/how-to-split-a-string-in-c/

+ Covert string:                                    https://www.programiz.com/cpp-programming/string-float-conversion                             
*/