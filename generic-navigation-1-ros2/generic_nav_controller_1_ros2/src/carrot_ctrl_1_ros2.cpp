#include <iostream>                     // IO operations

#include "rclcpp/rclcpp.hpp"            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/twist.hpp"  // Always hpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#define PI 3.14159265

using namespace std::chrono_literals;

    class carrot_ctrl_node : public rclcpp::Node
    {

        private:
            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //

            //nav_msgs::msg::Odometry pose_goal;
            //nav_msgs::msg::Odometry pose_msgs;

            typedef double PoseData;
            PoseData pose_curr[6];
            PoseData pose_goal[6];
            PoseData pose_tolerance[6] = {0.01, 0.01, 0.01, 0.01, 0.01, 0.1};
            std_msgs::msg::Bool in_process;
            geometry_msgs::msg::Twist command_vel;

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
            void pose_curr_sub_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                pose_curr[0] = msg->pose.position.x;
                pose_curr[1] = msg->pose.position.y;
                //pose_curr[2] = msg->pose.position.z;
                //pose_curr[3] = msg->pose.orientation.x;
                //pose_curr[4] = msg->pose.orientation.y;
                pose_curr[5] = msg->pose.orientation.z;
            }

            void pose_goal_sub_callback (const geometry_msgs::msg::PoseStamped::SharedPtr msg)
            {
                pose_goal[0] = msg->pose.position.x;
                pose_goal[1] = msg->pose.position.y;
                // pose_goal[2] = msg->pose.position.z;
                // pose_goal[3] = msg->pose.orientation.x;
                // pose_goal[4] = msg->pose.orientation.y;
                // pose_goal[5] = msg->pose.orientation.z;

                in_process.data = true;
                ctrl_process_pub_->publish(in_process);
            }

            void ctrl_callback()
            {
                if (in_process.data)
                {
                    std::cout << "a" << std::endl;

                    if ( 
                        (abs(pose_goal[0] - pose_curr[0]) > pose_tolerance[0]) ||
                        (abs(pose_goal[1] - pose_curr[1]) > pose_tolerance[1]) 
                    )

                    {
                        std::cout << "c" << std::endl;
                        pose_goal[5] = atan2(pose_goal[1] - pose_curr[1], pose_goal[0] - pose_curr[0]);
                        if (pose_goal[5] < 0) pose_goal[5] = 2*PI + pose_goal[5];
                        if (pose_curr[5] < 0) pose_curr[5] = 2*PI + pose_curr[5];
                        std::cout << "pose goal: "<< pose_goal[5] <<" pose curr: " << pose_curr[5] << std::endl;
                        if (abs(pose_goal[5] - pose_curr[5]) > pose_tolerance[5])
                        {
                            if ((pose_goal[5] - pose_curr[5]) > 0) command_vel.angular.z = 0.2;
                            else command_vel.angular.z = -0.2;
                            command_vel.linear.x = 0;
                        }
                        else
                        {
                            command_vel.linear.x = 0.2;
                            command_vel.angular.z = 0.0;
                        }
                        cmd_vel_pub_->publish(command_vel);
                    }
                    else
                    {
                        std::cout << "b" << std::endl;
                        command_vel.linear.x = 0.0;
                        command_vel.angular.z = 0.0;
                        cmd_vel_pub_->publish(command_vel);
                        in_process.data = false;
                        ctrl_process_pub_->publish(in_process);
                    }

                }
            }

            
        public:

            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            carrot_ctrl_node() :  Node ("carrot_ctrl_node"), in_process()
            {
                // Pub & Sub
                pose_curr_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_pub", 10, std::bind(&carrot_ctrl_node::pose_curr_sub_callback, this, std::placeholders::_1));
                pose_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10, std::bind(&carrot_ctrl_node::pose_goal_sub_callback, this, std::placeholders::_1));
                ctrl_process_pub_ = this->create_publisher<std_msgs::msg::Bool>("ctrl_process", 10);
                cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

                /* Algorithm TIMER */
                ctrl_timer_ = this->create_wall_timer(
                10ms, std::bind(&carrot_ctrl_node::ctrl_callback, this));

                in_process.data = false;
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