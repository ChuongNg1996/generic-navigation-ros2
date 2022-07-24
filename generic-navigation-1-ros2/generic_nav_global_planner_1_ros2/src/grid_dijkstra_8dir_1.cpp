#include <iostream>                     // IO operations
#include <vector>                     //

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

            std_msgs::msg::Bool in_process_init;
            typedef geometry_msgs::msg::PoseStamped GoalType;
            GoalType sub_goal;
            std::vector<GoalType> sub_goals;
            int i = 0;
            int max = 5;

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
                in_process_init.data = msg->data;
            }
            void sub_goal_callback()
            {
                if (!in_process_init.data)
                {
                    if (i < max)
                    {
                        sub_goal_pub_->publish(sub_goals[i]);
                        //std::cout << i << std::endl;
                        i++;
                        in_process_init.data = true;
                    }
                }
            }

        public:
            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            global_planner_node() :  Node ("global_planner_node"), in_process_init(), sub_goal()
            {
                // Pub & Sub
                ctrl_process_sub_ = this->create_subscription<std_msgs::msg::Bool>("ctrl_process_init", 10, std::bind(&global_planner_node::ctrl_process_callback, this, std::placeholders::_1));
                //ctrl_process_pub_ = this->create_publisher<std_msgs::msg::Bool>("ctrl_process_ack", 10);
                sub_goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_sub_goal", 10);

                in_process_init.data = true;

                sub_goal.pose.position.x = 5.0;
                sub_goal.pose.position.y = 2.0;
                sub_goals.push_back(sub_goal);
                sub_goal.pose.position.x = 8.0;
                sub_goal.pose.position.y = 5.0;
                sub_goals.push_back(sub_goal);
                sub_goal.pose.position.x = 8.0;
                sub_goal.pose.position.y = -5.0;
                sub_goals.push_back(sub_goal);
                sub_goal.pose.position.x = 15.0;
                sub_goal.pose.position.y = -3.0;
                sub_goals.push_back(sub_goal);
                sub_goal.pose.position.x = 10.0;
                sub_goal.pose.position.y = -6.0;
                sub_goals.push_back(sub_goal);

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