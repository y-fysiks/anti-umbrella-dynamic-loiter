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
                "apriltag_pose", 10, std::bind(&DronePIDNode::apriltagCB, this, std::placeholders::_1));
        }

        void DronePIDNode::initPublishers() {
            velxy_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("desd_vel", 10);
        }

        void DronePIDNode::initTimers() {
            pid_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&DronePIDNode::pidTimerCB, this));
        }

        void DronePIDNode::apriltagCB(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg) {
            RCLCPP_INFO(this->get_logger(), "Received AprilTag Pose");

            for (auto detection : msg->detections) {
                RCLCPP_INFO(this->get_logger(), "Pose: %f, %f", detection.centre.x, detection.centre.y);
            }
            
        }

        void DronePIDNode::pidTimerCB() {
            
        }


    }
}