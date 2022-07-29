#include <iostream>                                     // IO operations
#include <vector>                                       // Vector container
#include <chrono>                                       // For time utilities
#include <fstream>                                      // Input file
#include <stdio.h>                                      // Standard IO operations
#include <string.h>                                     // String operations

#include "rclcpp/rclcpp.hpp"                            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;                   // For time utilities

    class global_planner_node : public rclcpp::Node
    {
        private:
            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //
            typedef double PoseData;
            std_msgs::msg::Bool in_process_init;        // Indicate whether lower controller is in process or not
            typedef geometry_msgs::msg::PoseStamped GoalType;   // Define common type for sub goals
            GoalType sub_goal;                          // a goal unit
            std::vector<GoalType> sub_goals;            // vector of sub goals
                
            int i = 0;                                  // Index of list of sub goals
            int max;                                    // Number of sub goals

            // ---------------------------------------------------------- //
            // ---------            Map Variables               --------- //
            // ---------------------------------------------------------- //
            std::ifstream indata;                       // File reader 
            std::vector<PoseData> map_storage_1D;       // Store read values as 1D map
            std::vector<std::vector<PoseData>> map_storage_2D;    // Store read values as 2D map
            std::vector<PoseData> map_row;                        // Arrange values as row to assign to map
            int rows = 100;                             // Height in pixel
            int cols = 100;                             // Width in pixel
            int index = 2;                              // Start reading map from index 2 bc the first two are rows and cols
            PoseData pixel_length;                      // Physical length of a pixel's side in (m)

            // ---------------------------------------------------------- //
            // ---------        Algorithm Variables             --------- //
            // ---------------------------------------------------------- //

            int row;                                    // row variable, for utilities
            int col;                                    // col variable, for utilities
            int init_cell[2] = {10,93};                  // index of initial position (in row, col)
            int goal_cell[2] = {70,50};                 // index of goal position (in row, col)
            int current_cell[2] = {-1,-1};              // index of current cell (in row, col)
            int next_cell[2] = {-1,-1};                 // index of next cell (in row, col)
            std::vector<std::vector<PoseData>> g_map;                  // store g(n) values
            float dist;                                   // distance from init_cell to a free cell
            float min_dist = 0;                           // minimum distance of a cell to init_cell
            float max_dist = 0;                           // maximum distance of a cell to init_cell
            int direction_num = 8;                      // Number of moving direction
            std::vector<int> visited_rows;                   // Store visited rows
            std::vector<int> visited_cols;                   // Store visited columns

            bool goal_set = 1;
            bool goal_pending = 0;

            // ---------------------------------------------------------- //
            // ---------            ROS Variables               --------- //
            // ---------------------------------------------------------- //
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_process_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pub_;
            rclcpp::TimerBase::SharedPtr sub_goal_timer_;
            rclcpp::TimerBase::SharedPtr path_planning_timer_;

            // ---------------------------------------------------------- //
            // ---------          Callback Funtions             --------- //
            // ---------------------------------------------------------- //

            void path_planning_callback ()
            {
                if (goal_set)
                {
                    // ---------------------------------------------------------- //
                    // ---------           Path Planning Prepare        --------- //
                    // ---------------------------------------------------------- //

                    // Check if initial position is valid
                    if (map_storage_2D[init_cell[0]][init_cell[1]] == 1)
                    {
                        std::cout << "Invalid initial cell. " << std::endl;
                        //return 0;
                    } 

                    // Check if goal position is valid
                    if (map_storage_2D[goal_cell[0]][goal_cell[1]] == 1)
                    {
                        std::cout << "Invalid goal cell. " << std::endl;
                        //return 0;
                    } 
                    else std::cout << "Valid goal cell value: " << map_storage_2D[goal_cell[0]][goal_cell[1]] << std::endl;
                    map_storage_2D[goal_cell[0]][goal_cell[1]] = 5; //Random UNIQUE values to distinct with the rest

                    // Calculating g(n) map
                    g_map = map_storage_2D;
                    for (auto i = 0; i < rows; i++)
                    {
                        for (auto j = 0; j < cols; j++)
                        {
                            if (g_map[i][j] != 1) 
                            {
                                dist = sqrt(pow(i-init_cell[0],2) + pow(j-init_cell[1],2));
                                g_map[i][j] = dist;
                                if (dist > max_dist) max_dist = dist; // Find the maximum distance
                            }
                            else g_map[i][j] = -1; //Transfrom occupied zone from 1 to -1 
                        }
                    }

                    // ---------------------------------------------------------- //
                    // ---------           Path Planning Algorithm      --------- //
                    // ---------------------------------------------------------- //

                    // Assign current cell as goal cell, make a reverse path to initial cell
                    current_cell[0] = goal_cell[0];
                    current_cell[1] = goal_cell[1];

                    // While current cell doesn't arrive at initial cell
                    while ((current_cell[0] != init_cell[0]) || (current_cell[1] != init_cell[1]))
                    {
                        min_dist = max_dist + 1; // Initialize minimum distance as maximum distance

                        for (int i = 0; i < direction_num; i++)
                        {
                            switch(i)
                            {
                                // each case is a direction
                                case 0:
                                {
                                    row = current_cell[0] - 1;  // Up
                                    col = current_cell[1];      // Same
                                    break;
                                }
                                case 1:
                                {
                                    row = current_cell[0] - 1;  // Up
                                    col = current_cell[1] - 1;  // Left
                                    break;
                                }
                                case 2:
                                {
                                    row = current_cell[0];      // Same
                                    col = current_cell[1] - 1;  // Left
                                    break;
                                }
                                case 3:
                                {
                                    row = current_cell[0] + 1;  // Down
                                    col = current_cell[1] - 1;  // Left
                                    break;
                                }
                                case 4:
                                {
                                    row = current_cell[0] + 1;  // Down
                                    col = current_cell[1];      // Same
                                    break;
                                }
                                case 5:
                                {
                                    row = current_cell[0] + 1;  // Down
                                    col = current_cell[1] + 1;  // Right
                                    break;
                                }
                                case 6:
                                {
                                    row = current_cell[0];      // Same
                                    col = current_cell[1] + 1;  // Right
                                    break;
                                }
                                case 7:
                                {
                                    row = current_cell[0] - 1;  // Up
                                    col = current_cell[1] + 1;  // RIght
                                    break;
                                }
                            }

                            // Check if direction has obstacle
                            if (g_map[row][col] != -1) 
                            {
                                // Check if direction has been visited, avoid looping
                                if ( ! ((find(visited_rows.begin(), visited_rows.end(), row) != visited_rows.end()) && (find(visited_cols.begin(), visited_cols.end(), col) != visited_cols.end())) )
                                {
                                    // Check if direction has minimum distance to initial position
                                    if (g_map[row][col] < min_dist) 
                                    {
                                        // Store the direction if minimum distance is confirmed
                                        min_dist = g_map[row][col];
                                        next_cell[0] = row;
                                        next_cell[1] = col;
                                    }
                                }
                            }

                        }

                        // Move to direction that has minimum distance
                        current_cell[0] = next_cell[0];
                        current_cell[1] = next_cell[1];

                        // Store the index of the cell
                        visited_rows.push_back(next_cell[0]);
                        visited_cols.push_back(next_cell[1]);


                    }   

                    // Show the visited cells
                    std::cout << "Visited cell: " << std::endl;
                    for (auto x : visited_rows) std::cout << " " << x;
                    std::cout << std::endl;
                    for (auto x : visited_cols) std::cout << " " << x;
                    std::cout << "\n\n";

                    // Plot the path
                    for (auto i = 0; i< visited_rows.size() - 1;i++)
                    {
                        map_storage_2D[visited_rows[i]][visited_cols[i]] = 4; //Random UNIQUE values to distinct with the rest
                    }
                    
                    std::cout << "The path plotted: " << std::endl;
                    // Display the map again.
                    for (auto i = 0; i < rows; i++)
                    {
                        for (auto j = 0; j < cols; j++)
                        {
                            std::cout << map_storage_2D[i][j] << " ";
                            index++;
                        }
                        std::cout << std::endl;
                    }
                }
            }

            // Receive from lower controller that no process is running - ready to send NEW SUB GOAL
            void ctrl_process_callback (const std_msgs::msg::Bool::SharedPtr msg)
            {
                in_process_init.data = msg->data;
            }

            // Check whether lower controller is free or not to send NEW SUB GOAL
            void sub_goal_callback()
            {
                if (sub_goals.size() > 0)
                {
                    // If in_process_init = FALSE, lower controller is NOT in a process
                    if (!in_process_init.data) 
                    {
                            // // Then send the sub goals
                            // sub_goal_pub_->publish(sub_goals[0]);
                            // //std::cout << i << std::endl;
                            // i++;
                            // // AUTO set the state of process = TRUE to prevent sending new goals
                            // // Waiting for lower controller to set in_process_init = TRUE is not reliable
                            // in_process_init.data = true;        
                    }
                }    
            }

        public:
            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            global_planner_node() :  Node ("global_planner_node"), in_process_init(), sub_goal(), sub_goals(), indata(), map_storage_1D(), map_storage_2D(), map_row(), index()
            {
                // Pub & Sub

                // Receive from lower controller that no process is running - ready to send NEW SUB GOAL
                ctrl_process_sub_ = this->create_subscription<std_msgs::msg::Bool>("ctrl_process_init", 10, std::bind(&global_planner_node::ctrl_process_callback, this, std::placeholders::_1));
                
                // Send NEW SUB GOAL
                sub_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10);

                in_process_init.data = true;

                //---------------------- READ FILES ----------------------// 
                
                // Read the file (Must be full path)
                indata.open("/home/chuong/ros2_ws/src/generic-navigation-ros2/generic-navigation-1-ros2/generic_nav_map_1_ros2/maps/100_100_room.txt");

                // Check if there is any data/file has been correctly read
                if(!indata) { // file couldn't be opened
                    std::cout  << "Error: file could not be opened" << std::endl;
                    exit(1);
                }
                int num;                                    // store read values from map
                
                // Read and invert value
                indata >> num;
                if (num != 0) num = 0;  
                else num = 1;
                
                // Store map in 1D format
                while ( !indata.eof() ) { // keep reading until end-of-file
                    map_storage_1D.push_back(num);
                    //max++;

                    // Read and invert value
                    indata >> num; // sets EOF flag if no value found
                    if (num != 0) num = 0;
                    else num = 1;
                }
                indata.close();

                // Store map in 2D format
                // Push the pixel to map.
                for (auto i = 0; i < rows; i++)
                {
                    for (auto j = 0; j < cols; j++)
                    {
                        map_row.push_back(map_storage_1D[index]);
                        index++;
                    }
                    map_storage_2D.push_back(map_row);
                    map_row.clear();
                }

                // Display the map to check.
                // std::cout << "DISPLAY THE MAP: " << "\n\n";
                // for (auto i = 0; i < rows; i++)
                // {
                //     for (auto j = 0; j < cols; j++)
                //     {
                //         std::cout << map_storage_2D[i][j] << " ";
                //         index++;
                //     }
                //     std::cout << std::endl;
                // }
                // std::cout << std::endl;

                std::cout << "Map Imported." << std::endl;
                //--------------------------------------------------------// 



                /* Algorithm TIMER */
                sub_goal_timer_ = this->create_wall_timer(
                10ms, std::bind(&global_planner_node::sub_goal_callback, this));

                path_planning_timer_ = this->create_wall_timer(
                10ms, std::bind(&global_planner_node::path_planning_callback, this));

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