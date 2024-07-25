#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "get_direction_interface/action/go_to_pose.hpp"

// 'GoToPose' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class GoToPose : public rclcpp::Node {
public:
    GoToPose() : Node("go_to_pose_node") {
        RCLCPP_INFO(this->get_logger(), "Node initialized to test custom action interface.");
    }

};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GoToPose>();
    rclcpp::spin(node);
    rclcpp::shutdown();

}