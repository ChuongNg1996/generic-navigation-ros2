//#include <memory>                       // Dynamic memory management
//#include <thread>                       // Threaded Programming
#include <iostream>                     // IO operations
#include <cmath>                        // Generic math operations
#include <chrono>                       // For time
#include <eigen3/Eigen/Dense>           // For matrix operation

#include "rclcpp/rclcpp.hpp"            // C++ wrapper of ros2 node class
#include "sensor_msgs/msg/imu.hpp"      // Get this from cmd: ros2 topic info /imu
#include "nav_msgs/msg/odometry.hpp"    // Get this from cmd: ros2 topic info /odom
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


using namespace std::chrono_literals;


    class ekf_node : public rclcpp::Node
    {
        /*
        General class structure:
            1. Members:
                + Member variables
                + Member functions.
            2. Constructor.
            3. Public, Private, Protected tags

        Structure of this class:
            1. Define (private) VARIABLES: General Variables; Sensor Variables; Algorithm Variables; ROS Variables .
            2. Define (private) callback functions. 
            3. Call CONSTRUCTOR (automatically called when an object of a class is created) for initialization. 
                + Init pub, sub.
                + Init variables.
        */

        private:

            // ---------------------------------------------------------- //
            // ---------          General Variables             --------- //
            // ---------------------------------------------------------- //
            
            geometry_msgs::msg::PoseStamped pose_msgs;
         
            //generic_nav_msgs_srv_action_1_ros2::msg::GenericPose pose_msgs;
            
            // Define common type of sensor data
            typedef double SensorData;

            // ---------------------------------------------------------- //
            // ---------          Sensor Variables              --------- //
            // ---------------------------------------------------------- //

            //---------------------- IMU ----------------------//
            // Acceleration
            SensorData DotDot_imu[3];
            // Velocity
            SensorData Dot_imu[3];
            // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
            SensorData Pose_RPY_imu[6];
            SensorData Pose_Quaternions_imu[7];

            //---------------------- GPS ----------------------//
            // Velocity
            SensorData Dot_gps[3];
            // Position
            SensorData Pose_gps[3];
            // Relative to Base
            SensorData Offset_gps[3];

            //---------------------- odom ----------------------// 
            // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
            SensorData Pose_RPY_odom[6];
            SensorData Pose_Quaternions_odom[7];
     
            //---------------------- Avaibility Check ----------------------//
            bool imu_ready, gps_ready, gps_vel_ready, odom_ready, ekf_ready;

            // ---------------------------------------------------------- //
            // ---------        Algorithm Variables             --------- //
            // ---------------------------------------------------------- //

            Eigen::MatrixXd x_update; 
            Eigen::MatrixXd x_prev;
            Eigen::MatrixXd x_curr_minus;
            Eigen::MatrixXd A;
            Eigen::MatrixXd B;
            Eigen::MatrixXd sys_input;

            Eigen::MatrixXd sub_p_update;
            Eigen::MatrixXd p_update;
            Eigen::MatrixXd p_prev;
            Eigen::MatrixXd p_curr_minus;
            Eigen::MatrixXd Q;

            Eigen::MatrixXd C;
            Eigen::MatrixXd R;
            Eigen::MatrixXd subK;
            Eigen::MatrixXd K;

            Eigen::MatrixXd y_meas;
            Eigen::MatrixXd I;

            // ---------------------------------------------------------- //
            // ---------            ROS Variables               --------- //
            // ---------------------------------------------------------- //
            rclcpp::TimerBase::SharedPtr ekf_timer_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
            rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;


            // ---------------------------------------------------------- //
            // ---------          Callback Funtions             --------- //
            // ---------------------------------------------------------- //
            void imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                if (ekf_ready)
                {
                    // To access members of a structure, use the dot operator. To access members of a structure through  a pointer, use the arrow operator.
                    // Pose_Quaternions_imu[3] = msg->orientation.x;
                    // Pose_Quaternions_imu[4] = msg->orientation.y;
                    // Pose_Quaternions_imu[5] = msg->orientation.z;
                    // Pose_Quaternions_imu[6] = msg->orientation.w;
                    
                    // Transform from Quaternion ro RPY
                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(msg->orientation, quat_tf);
                    tf2::Matrix3x3 m(quat_tf);
                    m.getRPY(Pose_RPY_imu[3], Pose_RPY_imu[4], Pose_RPY_imu[5]);

                    imu_ready = 1;
                    

                }
            }
            void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                if (ekf_ready)
                {
                    Pose_Quaternions_odom[0] = msg->pose.pose.position.x;
                    Pose_Quaternions_odom[1] = msg->pose.pose.position.y;
                    Pose_Quaternions_odom[2] = msg->pose.pose.position.z;
                    // Pose_Quaternions_odom[3] = msg->pose.pose.orientation.x;
                    // Pose_Quaternions_odom[4] = msg->pose.pose.orientation.y;
                    // Pose_Quaternions_odom[5] = msg->pose.pose.orientation.z;
                    // Pose_Quaternions_odom[6] = msg->pose.pose.orientation.w;

                    // Transform from Quaternion ro RPY
                    tf2::Quaternion quat_tf;
                    tf2::fromMsg(msg->pose.pose.orientation, quat_tf);
                    tf2::Matrix3x3 m(quat_tf);
                    m.getRPY(Pose_RPY_odom[3], Pose_RPY_odom[4], Pose_RPY_odom[5]);
                    
                    odom_ready = 1;
                }
            }
            void ekf_callback ()
            {
                if (imu_ready && odom_ready)
                {
                    ekf_ready = 0;
                    gps_ready = 0;
                    odom_ready = 0;
                    imu_ready = 0;

                    // Assign value for prediction state
                    sys_input << Pose_Quaternions_odom[0], Pose_Quaternions_odom[1], Pose_Quaternions_odom[2], Pose_RPY_odom[3], Pose_RPY_odom[4], Pose_RPY_odom[5];

                    // Compute Priori
                    x_curr_minus = A*x_prev + B*sys_input;
                    // [6x1] = [6x6]*[6x1] + [6x6]*[6x1]
                    p_curr_minus = A*p_prev*A.transpose() + Q;
                    // [6x6] = [6x6]*[6x6]*[6x6] + [6x6]

                    // Kalman filter gain

                    subK = C*p_curr_minus*C.transpose() + R;
                    // [6x6] = [6x6]*[6x6]*[6x6] + [6x6]
                    K = p_curr_minus*C.transpose()*subK.inverse();
                    // [6x6] = [6x6]*[6x6]*[6x6]
                    
                    // Assign value for correction state
                    y_meas << Pose_Quaternions_odom[0], Pose_Quaternions_odom[1], Pose_Quaternions_odom[2], Pose_RPY_imu[3], Pose_RPY_imu[4], Pose_RPY_imu[5]; // Must change to gps
                    //y_meas << xChange_gps, yChange_gps, zChange_gps, phiChange_imu, thetaChange_imu, psiChange_imu;

                    x_update = x_curr_minus + K*(y_meas - C*x_curr_minus);

                    sub_p_update = (I - K*C);
                    p_update = (I - K*C)*p_curr_minus*sub_p_update.transpose() + K*R*K.transpose();
                    
                    // Update the values
                    x_prev = x_update;
                    p_prev = p_update;

                    // Publish the updated values to other nodes.
                    pose_msgs.pose.position.x = x_update(0);
                    pose_msgs.pose.position.y = x_update(1);
                    pose_msgs.pose.position.z = x_update(2);
                    pose_msgs.pose.orientation.x = x_update(3);
                    pose_msgs.pose.orientation.y = x_update(4);
                    pose_msgs.pose.orientation.z = x_update(5);

                    pose_pub_->publish(pose_msgs);

                    // Ready to read sensor again.
                    ekf_ready = 1;

                    // std::cout << "Measurement Result: " << y_meas.transpose() << std::endl;
                    // std::cout << "Odom Result: " << sys_input.transpose() << std::endl;
                    // std::cout << "K: " << std::endl;
                    // std::cout << K << std::endl;
                    // std::cout << "EKF Result: " << x_prev.transpose() << std::endl << std::endl << std::endl;
                }
            }

        public:
            // ---------------------------------------------------------- //
            // ---------             CONSTRUCTOR                --------- //
            // ---------------------------------------------------------- //

            // CONSTRUCTOR: a special method that is automatically called when an object of a class is created.
            ekf_node() :    Node ("ekf_node"),
                            imu_ready(), gps_ready(), gps_vel_ready(), odom_ready(), ekf_ready(),
                            x_update(6,1), x_prev(6,1), x_curr_minus(6,1), A(6,6), B(6,6), sys_input(6,1),
                            sub_p_update(6,6), p_update(6,6), p_prev(6,6), p_curr_minus(6,6), Q(6,6),
                            C(6,6), R(6,6), subK(6,6), K(6,6), y_meas(6,1), I(6,6)
                            
            {
                // Pub & Sub
                imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ekf_node::imu_callback, this, std::placeholders::_1));
                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ekf_node::odom_callback, this, std::placeholders::_1));
                pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_pub", 10);


                /* Algorithm TIMER */
                ekf_timer_ = this->create_wall_timer(
                100ms, std::bind(&ekf_node::ekf_callback, this));

                imu_ready = 0; gps_ready = 0; gps_vel_ready = 0; odom_ready = 0; ekf_ready = 1;
                // ---------------------------------------------------------- //
                // ---------        Algorithm Variables             --------- //
                // ---------------------------------------------------------- //

                x_prev << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

                A <<    0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0,
                0,  0,  0,  0,  0,  0;

                B <<    1,  0,  0,  0,  0,  0,
                0,  1,  0,  0,  0,  0,
                0,  0,  1,  0,  0,  0,
                0,  0,  0,  1,  0,  0,
                0,  0,  0,  0,  1,  0,
                0,  0,  0,  0,  0,  1;

                p_prev <<   pow(10,-5), 0, 0, 0, 0, 0,
                0, pow(10,-5), 0, 0, 0, 0,
                0, 0, pow(10,-5), 0, 0, 0,
                0, 0, 0, pow(10,-5), 0, 0,
                0, 0, 0, 0, pow(10,-5), 0,
                0, 0, 0, 0, 0, pow(10,-5);

                Q << pow(10,-5), 0, 0, 0, 0, 0,
                0, pow(10,-5), 0, 0, 0, 0,
                0, 0, pow(10,-5), 0, 0, 0,
                0, 0, 0, pow(10,-5), 0, 0,
                0, 0, 0, 0, pow(10,-5), 0,
                0, 0, 0, 0, 0, pow(10,-5);

                C << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;

                R << 0.002501, 0, 0, 0, 0, 0,
                0, 0.002501, 0, 0, 0, 0,
                0, 0, 0.002501, 0, 0, 0,
                0, 0, 0, 0.002501, 0, 0,
                0, 0, 0, 0, 0.002501, 0,
                0, 0, 0, 0, 0, 0.002501;

                I << 1, 0, 0, 0, 0, 0,
                0, 1, 0, 0, 0, 0,
                0, 0, 1, 0, 0, 0,
                0, 0, 0, 1, 0, 0,
                0, 0, 0, 0, 1, 0,
                0, 0, 0, 0, 0, 1;  

            }
    };


// ---------------------------------------------------------- //
// ---------                MAIN                    --------- //
// ---------------------------------------------------------- //

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ekf_node>());
    rclcpp::shutdown();
    return 0;
}
/*
+ ERROR #1: https://get-help.robotigniteacademy.com/t/unable-to-import-sensor-msgs-in-ros2/8185 
-> Repalce #include “sensor_msgs/msg/Imu.hpp” with #include “sensor_msgs/msg/imu.hpp”

+ EEROR #2: expected identifier before numeric constant 
-> https://stackoverflow.com/questions/11490988/c-compile-time-error-expected-identifier-before-numeric-constant, SOLUTION:
    class Foo {
        vector<string> name;
        vector<int> val;
    public:
        Foo() : name(5), val(5,0) {}
    };

+ Constructor: https://www.w3schools.com/cpp/cpp_constructors.asp 

+ (.) vs. (->): https://www.tutorialspoint.com/cplusplus/cpp_member_operators.htm To access members of a structure, use the 
dot operator. To access members of a structure through  a pointer, use the arrow operator.

+ ROS2: From Quaternion ro RPY https://gist.github.com/simutisernestas/14047512cbffd355a5c29d0c4cbf0eb5 
+ ROS1: From Quaternion ro RPY https://gist.github.com/marcoarruda/f931232fe3490b7fa20dbb38da1195ac 
*/
