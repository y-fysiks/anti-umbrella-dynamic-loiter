#pragma once

// ROS2 Requirements
#include <rclcpp/rclcpp.hpp>

// System Requirements
#include <chrono>
#include <fstream>
#include <functional>
#include <sstream>   // std::stringstream
#include <stdexcept> // std::runtime_error
#include <string>
#include <vector>

//Message Requirements
#include <geometry_msgs/msg/twist.hpp>
#include <apriltag_msgs/msg/april_tag_detection_array.hpp>


namespace anti_umbrella {
    namespace dronepid {
        
        class DronePIDNode : public rclcpp::Node {
            public:
                DronePIDNode();

            private:
                void initParams();

                void initSubscribers();

                void initPublishers();

                void initTimers();

                void apriltagCB(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg);

                void pidTimerCB();

                //Global variables

                double kp_;
                double ki_;
                double kd_;

                rclcpp::TimerBase::SharedPtr pid_timer_;


                //ROS2 Publishers
                rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velxy_pub_;

                //ROS2 Subscribers
                rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_sub_;
        } ;
    }
}
