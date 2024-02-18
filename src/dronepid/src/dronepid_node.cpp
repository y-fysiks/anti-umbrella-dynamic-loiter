#include <dronepid_node.hpp>

namespace anti_umbrella {
    namespace dronepid {
        DronePIDNode::DronePIDNode() : Node("drone_pid_node") {
            RCLCPP_INFO(this->get_logger(), "Drone PID Node Launched");

            this->initParams();
            this->initSubscribers();
            this->initPublishers();
            this->initTimers();

        }

        void DronePIDNode::initParams() {
            this->declare_parameter("kp", 0.1);
            this->declare_parameter("ki", 0.1);
            this->declare_parameter("kd", 0.1);
        }

        void DronePIDNode::initSubscribers() {
            apriltag_sub_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
                "/detections", 10, std::bind(&DronePIDNode::apriltagCB, this, std::placeholders::_1));
        }

        void DronePIDNode::initPublishers() {
            velxy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("desd_vel", 10);
        }

        void DronePIDNode::initTimers() {
            pid_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DronePIDNode::pidTimerCB, this));
        }

        void DronePIDNode::apriltagCB(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
            // RCLCPP_INFO(this->get_logger(), "Received AprilTag Pose");

            for (auto detection : msg->detections) {
                RCLCPP_INFO(this->get_logger(), "Pose: %f, %f", detection.centre.x, detection.centre.y);
                RCLCPP_INFO(this->get_logger(), "Points: x1: %f, y1:%f", detection.corners[0].x, detection.corners[0].y);
                RCLCPP_INFO(this->get_logger(), "        x2: %f, y2:%f", detection.corners[1].x, detection.corners[1].y);
                RCLCPP_INFO(this->get_logger(), "        x3: %f, y3:%f", detection.corners[2].x, detection.corners[2].y);
                RCLCPP_INFO(this->get_logger(), "        x4: %f, y4:%f", detection.corners[3].x, detection.corners[3].y);
            }
            
        }

        void DronePIDNode::pidTimerCB() {
            
        }


    }
}