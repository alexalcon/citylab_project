#include "rclcpp/rclcpp.hpp"
<<<<<<< HEAD
#include "rclcpp_action/rclcpp_action.hpp"
=======
>>>>>>> 7cb1fb76d838683e7f1385f91fc6ee71cf54c5a7
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "get_direction_interface/action/go_to_pose.hpp"
<<<<<<< HEAD
=======
#include "rclcpp/utilities.hpp"
>>>>>>> 7cb1fb76d838683e7f1385f91fc6ee71cf54c5a7

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