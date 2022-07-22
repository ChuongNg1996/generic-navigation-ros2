#include <iostream>                     // IO operations

#include "rclcpp/rclcpp.hpp"            // C++ wrapper of ros2 node class
#include "std_msgs/msg/bool.hpp" 
#include "geometry_msgs/msg/twist.hpp"  // Always hpp
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;

    class global_planner_node : public rclcpp::Node
    {
        private:
            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //

            std_msgs::msg::Bool in_process;
            geometry_msgs::msg::PoseStamped sub_goal;

            // ---------------------------------------------------------- //
            // ---------            ROS Variables               --------- //
            // ---------------------------------------------------------- //
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr ctrl_process_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sub_goal_pub_;
            rclcpp::TimerBase::SharedPtr sub_goal_timer_;

            // ---------------------------------------------------------- //
            // ---------          Callback Funtions             --------- //
            // ---------------------------------------------------------- //
            void ctrl_process_callback (const std_msgs::msg::Bool::SharedPtr msg)
            {
                in_process.data = msg->data;
            }
            void sub_goal_callback()
            {
                if (!in_process.data)
                {
                    sub_goal_pub_->publish(sub_goal);
                }
            }


        public:
            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            global_planner_node() :  Node ("global_planner_node"), in_process(), sub_goal()
            {
                // Pub & Sub
                ctrl_process_sub_ = this->create_subscription<std_msgs::msg::Bool>("ctrl_process", 10, std::bind(&global_planner_node::ctrl_process_callback, this, std::placeholders::_1));
                sub_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10);

                /* Algorithm TIMER */
                sub_goal_timer_ = this->create_wall_timer(
                100ms, std::bind(&global_planner_node::sub_goal_callback, this));

                in_process.data = false;
                sub_goal.pose.position.x = 10.0;
                sub_goal.pose.position.y = 10.0;


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