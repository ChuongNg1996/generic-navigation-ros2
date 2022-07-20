#include <memory> // Dynamic memory management
#include <thread> 
#include <iostream>
#include <cmath>
#include <chrono>
#include <eigen3/Eigen/Dense>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"  // Get this from cmd: ros2 topic info /imu
#include "nav_msgs/msg/odometry.hpp"    // Get this from cmd: ros2 topic info /odom

using namespace std::chrono_literals;


    class ekf_node : public rclcpp::Node
    {
        public:
            // Define common type of sensor data
            typedef float SensorData;

            // ---------------------------------------------------------- //
            // ---------          Sensor Variables              --------- //
            // ---------------------------------------------------------- //

            //---------------------- IMU ----------------------//
            // Acceleration
            SensorData xDotDot_imu, yDotDot_imu, zDotDot_imu;
            // Velocity
            SensorData xDot_imu, yDot_imu, zDot_imu;
            // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
            SensorData xChange_imu, yChange_imu, zChange_imu, phiChange_imu, thetaChange_imu, psiChange_imu;

            //---------------------- GPS ----------------------//
            // Velocity
            SensorData xVel_gps, yVel_gps, zVel_gps;
            // Position
            SensorData xChange_gps, yChange_gps, zChange_gps;
            // Relative to Base
            SensorData xOffset_gps, yOffset_gps, zOffset_gps;


            //---------------------- odom ----------------------// 
            // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
            SensorData xChange_odom, yChange_odom, zChange_odom, phiChange_odom, thetaChange_odom, psiChange_odom ;

            //---------------------- Sensor Fusion ----------------------//
            SensorData xChange_final, yChange_final, zChange_final, phiChange_final, thetaChange_final, psiChange_final;
            
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

            // CONSTRUCTOR: a special method that is automatically called when an object of a class is created.
            ekf_node() :    Node ("ekf_node"),
                            xDotDot_imu(), yDotDot_imu(), zDotDot_imu(), xDot_imu(), yDot_imu(), zDot_imu(), xChange_imu(), yChange_imu(), zChange_imu(), phiChange_imu(), thetaChange_imu(), psiChange_imu(),
                            xVel_gps(), yVel_gps(), zVel_gps(), xChange_gps(), yChange_gps(), zChange_gps(), xOffset_gps(), yOffset_gps(), zOffset_gps(),
                            xChange_odom(), yChange_odom(), zChange_odom(), phiChange_odom(), thetaChange_odom(), psiChange_odom(), 
                            xChange_final(), yChange_final(), zChange_final(), phiChange_final(), thetaChange_final(), psiChange_final(),
                            imu_ready(), gps_ready(), odom_ready(),
                            x_update(6,1), x_prev(6,1), x_curr_minus(6,1), A(6,6), B(6,6), sys_input(6,1),
                            sub_p_update(6,6), p_update(6,6), p_prev(6,6), p_curr_minus(6,6), Q(6,6),
                            C(6,6), R(6,6), subK(6,6), K(6,6), y_meas(6,1), I(6,6)
                            
            {
                imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 10, std::bind(&ekf_node::imu_callback, this, std::placeholders::_1));
                odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&ekf_node::odom_callback, this, std::placeholders::_1));

                /*Next, timer_ is initialized, which causes the timer_callback function to be executed twice a second.*/
                timer_ = this->create_wall_timer(
                100ms, std::bind(&ekf_node::ekf_callback, this));


                // ---------------------------------------------------------- //
                // ---------          Sensor Variables              --------- //
                // ---------------------------------------------------------- //

                //---------------------- IMU ----------------------//
                // Acceleration
                xDotDot_imu = 0; yDotDot_imu = 0; zDotDot_imu = 0;
                // Velocity
                xDot_imu = 0; yDot_imu = 0; zDot_imu = 0;
                // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
                xChange_imu = 0; yChange_imu = 0; zChange_imu = 0; phiChange_imu = 0; thetaChange_imu = 0; psiChange_imu = 0;

                //---------------------- GPS ----------------------//
                // Velocity
                xVel_gps = 0; yVel_gps = 0; zVel_gps = 0;
                // Position
                xChange_gps = 0; yChange_gps = 0; zChange_gps = 0;
                // Relative to Base
                xOffset_gps = 0; yOffset_gps = 0; zOffset_gps = 0;


                //---------------------- odom ----------------------// 
                // Position & Orientation (x,y,z,Roll,Pitch,Yaw)
                xChange_odom = 0; yChange_odom = 0; zChange_odom = 0; phiChange_odom = 0; thetaChange_odom = 0; psiChange_odom = 0;

                //---------------------- Sensor Fusion ----------------------//
                xChange_final = 0; yChange_final = 0; zChange_final = 0; phiChange_final = 0; thetaChange_final = 0; psiChange_final = 0;

                //--- Avaibility Check ---//
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
        
        private:
            void imu_callback (const sensor_msgs::msg::Imu::SharedPtr msg)
            {
                if (ekf_ready)
                {
                    // To access members of a structure, use the dot operator. To access members of a structure through  a pointer, use the arrow operator.
                    phiChange_imu = msg->orientation.x;
                    thetaChange_imu = msg->orientation.y;
                    psiChange_imu = msg->orientation.z;
                    imu_ready = 1;
                }
            }
            void odom_callback (const nav_msgs::msg::Odometry::SharedPtr msg)
            {
                if (ekf_ready)
                {
                    xChange_odom = msg->pose.pose.position.x;
                    yChange_odom = msg->pose.pose.position.y;
                    zChange_odom = msg->pose.pose.position.z;
                    phiChange_odom = msg->pose.pose.orientation.x;
                    thetaChange_odom = msg->pose.pose.orientation.y;
                    psiChange_odom = msg->pose.pose.orientation.z;
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

                    sys_input << xChange_odom, yChange_odom, zChange_odom, phiChange_odom, thetaChange_odom, psiChange_odom;

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
                    
                    // Update
                    y_meas << xChange_odom, yChange_odom, zChange_odom, phiChange_imu, thetaChange_imu, psiChange_imu;

                    x_update = x_curr_minus + K*(y_meas - C*x_curr_minus);

                    sub_p_update = (I - K*C);
                    p_update = (I - K*C)*p_curr_minus*sub_p_update.transpose() + K*R*K.transpose();

                    x_prev = x_update;
                    p_prev = p_update;

                    ekf_ready = 1;

                    std::cout << "Measurement Result: " << y_meas.transpose() << std::endl;
                    std::cout << "Odom Result: " << sys_input.transpose() << std::endl;
                    std::cout << "K: " << std::endl;
                    std::cout << K << std::endl;
                    std::cout << "EKF Result: " << x_prev.transpose() << std::endl << std::endl << std::endl;
                }
            }
            rclcpp::TimerBase::SharedPtr timer_;
            rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
            rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    };

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

*/
