#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "get_direction_interface/action/go_to_pose.hpp"

using GoToPose = get_direction_interface::action::GoToPose;
using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;
using namespace std::placeholders;

// 'GoToPose' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class GoToPose : public rclcpp::Node {
// odometry sub, vel pub and action server node interface
// class node constructor
public:
    explicit GoToPose(const rclcpp::NodeOptions& options = rclcpp::NodeOptions()) 
        : Node("go_to_pose_node", options) {

        // action server initialization
        this->action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(GoToPose::handleGoal, this, _1, _2),
            std::bind(GoToPose::handleCancel, this, _1),
            std::bind(GoToPose::handleAccepted, this, _1));   
    }

// odometry sub, vel pub and action server node implementation details
private:
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pubisher_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    
    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;

    // action server related methods
    //-------------------------------------------------------------------------------    
    rclcpp_action::GoalResponse handleGoal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GoToPose::Goal> goal) {
        
        RCLCPP_INFO(this->get_logger(), "Received goal request: 
            /n/tx: %f 
            /n/ty: %f 
            /n/ttheta: %f", 
            goal->goal_pos.x,
            goal->goal_pos.y,
            goal->goal_pos.theta;
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handleCancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handleAccepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        // this needs to return quickly to avoid blocking the 
        // executor, so spin up a new thread
        std::thread{std::bind(&GoToPose::execute, this, _1), goal_handle}.detach();    
    }

    void execute(const sts::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal");
    }
    //-------------------------------------------------------------------------------    
}; // end class GoToPose

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    
    auto action_server = std::make_shared<GoToPose>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(action_server);
    executor.spin();

    rclcpp::shutdown();

    return 0;
} // end function main