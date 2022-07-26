#include <iostream>                                     // IO operations
#include <chrono>                                       // For time utilities

#include "rclcpp/rclcpp.hpp"                            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/twist.hpp"                  // Always hpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265

using namespace std::chrono_literals;                   // For time utilities

    class carrot_ctrl_node : public rclcpp::Node
    {

        private:

            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //

            typedef double PoseData;                    // General data type for robot pose
            PoseData pose_curr[6];                      // Current pose read from LOCALIZATION node
            PoseData pose_goal[6];                      // Goal pose received from GLOBAL PATH PLANNER node
            std_msgs::msg::Bool in_process_init;        // Inform GLOBAL PATH PLANNER whether controller is running or not
                                                        // So that it sends new goal
            geometry_msgs::msg::Twist command_vel;      // Command body velocity to robot frame

            // ---------------------------------------------------------- //
            // ---------        Algorithm Variables             --------- //
            // ---------------------------------------------------------- //

            PoseData pose_tolerance[6] = {0.1, 0.1, 0.1, 0.1, 0.1, 0.1}; // Pose tolerance
            PoseData angle_distance_right, angle_distance_left; // Arc length from curren heading to goal heading in both directions
            double linear_vel = 0.5;                    // Kind of inversed-proportional to tolerance
            double angular_vel = 0.2;                   // Kind of inversed-proportional to tolerance
            bool pose_updated = 0;                      // Check whether new pose is updated, to avoid using old pose without update from LOCALIZATION node
            

            // ---------------------------------------------------------- //
            // ---------            ROS Variables               --------- //
            // ---------------------------------------------------------- //
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_curr_sub_;    
            rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_goal_sub_;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr ctrl_process_pub_;
            rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
            rclcpp::TimerBase::SharedPtr ctrl_timer_;


            // ---------------------------------------------------------- //
            // ---------          Callback Funtions             --------- //
            // ---------------------------------------------------------- //

            // Read current pose from LOCALIZATION node
            void pose_curr_sub_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                pose_curr[0] = msg->pose.position.x;
                pose_curr[1] = msg->pose.position.y;
                pose_curr[5] = msg->pose.orientation.z;
                pose_updated = 1;
            }

            // Read sub goals from GLOBAL PATH PLANNER node
            void pose_goal_sub_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                pose_goal[0] = msg->pose.position.x;
                pose_goal[1] = msg->pose.position.y;
                // pose_goal[5] = msg->pose.orientation.z;

                // Inform to GLOBAL PATH PLANNER that controller is initialized
                in_process_init.data = true;
                ctrl_process_pub_->publish(in_process_init);
            }

            // Perform algorithm
            void ctrl_callback()
            {
                // If sub goal is obtained and NOT yet reached, activate controller
                if (in_process_init.data)
                {
                    // The algorithm performs if the position is updated from LOCALIZATION Node to avoid using old value
                    if (pose_updated) 
                    {
                        // ---------------------------------------------------------- //
                        // ---------                Algorithm               --------- //
                        // ---------------------------------------------------------- //

                        // If current position is NOT near sub goal (within tolerance)
                        if ( 
                            (abs(pose_goal[0] - pose_curr[0]) > pose_tolerance[0]) ||
                            (abs(pose_goal[1] - pose_curr[1]) > pose_tolerance[1]) 
                        )
                        {

                            /*
                            Carrot Algorithm:
                                + Constantly check and correct heading (to the sub goal) first 
                                    -> Normalize heading from 0 to 2*PI (heading direction is counterclockwise)
                                    -> If the heading is not reached, check whether required heading is larger or smaller than current heading.
                                        -> If required heading is LARGER, then ROTATING RIGHT is the sum of (1)[magnitude of current heading] and 
                                        (2)[offset from 2*PI to required heading]; ROTATING LEFT is the offset from required heading to current
                                        heading.
                                        -> If required heading is SMALLER, vice versa.
                                            -> If ROTATING RIGHT is shorter, then rotate right and vice versa.   
                                    (There are A LOT OF WAYS to implement the above logic)

                                + Once heading is near enough, move forward till near enough.
                                    -> Since heading is constantly checked, moving forward will be stopped to prioritize for heading correction
                                    if the heading deviation exceed the tolerance.
                            */

                            // Normalize headings from 0 to 2*PI (heading direction is counterclockwise)
                            pose_goal[5] = atan2(pose_goal[1] - pose_curr[1], pose_goal[0] - pose_curr[0]);
                            if (pose_goal[5] < 0) pose_goal[5] = 2*PI + pose_goal[5];
                            if (pose_curr[5] < 0) pose_curr[5] = 2*PI + pose_curr[5];

                            //std::cout << "pose goal[5]: "<< pose_goal[5] <<" pose curr[5]: " << pose_curr[5] << std::endl;

                            // Constantly check and correct heading (to the sub goal) first 
                            if (abs(pose_goal[5] - pose_curr[5]) > pose_tolerance[5])
                            {
                                // Check whether required heading is larger or smaller than current heading.
                                if (pose_goal[5] > pose_curr[5])
                                {
                                    // If required heading is LARGER, then ROTATING RIGHT is the sum of (1)[magnitude of current heading] and 
                                    // (2)[offset from 2*PI to required heading]; ROTATING LEFT is the offset from required heading to current
                                    // heading.
                                    angle_distance_right = abs((2*PI - pose_goal[5]) + pose_curr[5]); 
                                    angle_distance_left = abs(pose_goal[5] - pose_curr[5]); 
                                }
                                else    // There is NO EQUAL case since that would skip this algorithm
                                {
                                    angle_distance_right = abs(pose_curr[5] - pose_goal[5]);
                                    angle_distance_left = abs((2*PI - pose_curr[5]) + pose_goal[5]);                                 
                                }
                                // If ROTATING RIGHT is shorter, then rotate right and vice versa.   
                                if (angle_distance_right < angle_distance_left) command_vel.angular.z = -angular_vel;
                                else command_vel.angular.z = angular_vel;
                                command_vel.linear.x = 0;
                            }
                            else
                            {
                                // Once heading is near enough, move forward till near enough.
                                command_vel.linear.x = linear_vel;
                                command_vel.angular.z = 0.0;
                            }
                            // Command body velocity to robot.
                            cmd_vel_pub_->publish(command_vel);
                        }
                        else
                        {
                            // If current position is near sub goal (within tolerance)
                            command_vel.linear.x = 0.0;
                            command_vel.angular.z = 0.0;
                            // Command robot to stop
                            cmd_vel_pub_->publish(command_vel);
                            // Complete the process and inform this to GLOBAL PATH PLANNER on "ctrl_process_init" topic
                            in_process_init.data = false;
                            ctrl_process_pub_->publish(in_process_init);
                            //std::cout << "Sub Goal Reached" << std::endl;
                        }
                    }
                    pose_updated = 0;
                }
                else
                {
                    // If there is not sub goal, inform this to GLOBAL PATH PLANNER on "ctrl_process_init" topic
                    ctrl_process_pub_->publish(in_process_init);
                }
            }
            
        public:

            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            carrot_ctrl_node() :  Node ("carrot_ctrl_node"), in_process_init()
            {
                // Pub & Sub

                // Sub to current pose topic of LOCALIZATION node
                pose_curr_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_pub", 10, std::bind(&carrot_ctrl_node::pose_curr_sub_callback, this, std::placeholders::_1));

                // Sub to sub goal topic of GLOBAL PATH PLANNER node
                pose_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10, std::bind(&carrot_ctrl_node::pose_goal_sub_callback, this, std::placeholders::_1));

                // Inform GLOBAL PATH PLANNER node whether the controller is in process or not
                ctrl_process_pub_ = this->create_publisher<std_msgs::msg::Bool>("ctrl_process_init", 10);

                // Command body velocity to robot
                cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

                /* Algorithm TIMER */
                ctrl_timer_ = this->create_wall_timer(
                10ms, std::bind(&carrot_ctrl_node::ctrl_callback, this));

                in_process_init.data = false;
            }

    };

// ---------------------------------------------------------- //
// ---------                MAIN                    --------- //
// ---------------------------------------------------------- //

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<carrot_ctrl_node>());
    rclcpp::shutdown();
    return 0;
}
/*
* atan2: https://cplusplus.com/reference/cmath/atan2/ returns the principal value of the arc tangent of y/x, expressed in radians.
* by default, callback functions in a single node don't have separate threads and have to wait for each other to complete.
*/