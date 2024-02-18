#include <dronepid_node.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<anti_umbrella::dronepid::DronePIDNode>());
    rclcpp::shutdown();
    return 0;
}