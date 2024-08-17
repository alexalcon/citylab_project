#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <action_msgs/msg/goal_status_array.hpp>
#include <action_msgs/srv/cancel_goal.hpp>
#include "rclcpp/timer.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "get_direction_interface/action/go_to_pose.hpp"

#include <iostream>
#include <chrono>
#include <functional>

using namespace std::chrono_literals;

// 'GoToPoseActionServer' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class GoToPoseActionServer : public rclcpp::Node {
// action server, odom sub and vel pub node interface
// class node constructor
public: 
    // renaming custom action msg structure
    using GoToPose = get_direction_interface::action::GoToPose;
    // renaming class template (ServerGoalHandle) structure to GoalHandleGoToPose
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<GoToPose>;  

    GoToPoseActionServer() : Node("go_to_pose_node") {
        callback_group_vel_pub = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_odom_sub = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        action_server_ = rclcpp_action::create_server<GoToPose>(
            this,
            "go_to_pose",
            std::bind(&GoToPoseActionServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&GoToPoseActionServer::handle_cancel, this, std::placeholders::_1),
            std::bind(&GoToPoseActionServer::handle_accepted, this, std::placeholders::_1));

        rclcpp::SubscriptionOptions odom_sub_options;
        odom_sub_options.callback_group = callback_group_odom_sub;
        odom_subscriber_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&GoToPoseActionServer::odom_callback, this, std::placeholders::_1),
            odom_sub_options);

        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&GoToPoseActionServer::velocityControlLoop, this), callback_group_vel_pub);
    }
// action server, odom sub and vel pub node implementation details
private:
    // GoToPoseActionServer node data members
    //---------------------------------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr callback_group_vel_pub;
    rclcpp::CallbackGroup::SharedPtr callback_group_odom_sub;

    // objects that manage the lifecycle and interactions with 
    // the underlying ROS2 communication infrastructure
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    // pose variables for robot pose control 
    geometry_msgs::msg::Pose2D desired_pos_;
    geometry_msgs::msg::Pose2D current_pos_;
   
    // additional data memebers
    geometry_msgs::msg::Twist vel_data;
    //---------------------------------------------------------------------------
    
    // GoToPoseActionServer node function members
    //-------------------------------------------------------------------------------------------
    // action goal handle receiving
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const GoToPose::Goal> goal) {
        
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), 
            "Received goal request with position x: %f, y: %f, theta: %f",
            goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
            desired_pos_ = goal->goal_pos;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    // action goal cancelation handling 
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal.");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    // action goal execution handling 
    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        using namespace std::placeholders;
        // this needs to return quickly to avoid blocking the executor, so spin up a new thread
        std::thread{std::bind(&GoToPoseActionServer::execute, this, _1), goal_handle}.detach();
    }

    // action goal execution
    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        auto feedback = std::make_shared<GoToPose::Feedback>();
        auto result = std::make_shared<GoToPose::Result>();
        
        // main pose control/acting loop   
        /**
         * In this control/acting loop, the decision making algorithm 
         * logic is performed so that velocity commands are published.
         * 
         * Some mathematical concepts are used in this control/acting  
         * loop for navigating the robot to the desired 2D pose.
         */
        rclcpp::Rate loop_rate(10);
        while (rclcpp::ok()) {
            // decision making control commands computation
            double diff_x = desired_pos_.x - current_pos_.x;
            double diff_y = desired_pos_.y - current_pos_.y;
            double distance = sqrt(diff_x * diff_x + diff_y * diff_y); // euclidean distance formula
            double angle_to_goal = atan2(diff_y, diff_x);
            double angle_diff = (angle_to_goal * 180.0 / M_PI) - current_pos_.theta; // degrees
            // std::cout << "desired angle: " << desired_pos_.theta << std::endl;
            // std::cout << "diff_x: " << diff_x << " - diff_y: " << diff_y << std::endl;
            // std::cout << "angle_to_goal: " << angle_to_goal * 180.0 / M_PI << " - angle_diff: " << angle_diff << std::endl;
            
            // decision making algorithm logic
            // to publish velocity commands
            if (distance > 0.05) {                                   
                vel_data.linear.x = 0.2;                          // fixed linear velocity
                vel_data.angular.z = (angle_diff * M_PI / 180.0); // adjust angular velocity (radian values)

                feedback->current_pos = current_pos_; // action feedback data
                goal_handle->publish_feedback(feedback);

            } else {
                vel_data.linear.x = 0.0;
                vel_data.angular.z = 0.0;
                
                feedback->current_pos = current_pos_;  // action feedback data
                goal_handle->publish_feedback(feedback);

                bool flag = true;
                while (flag) {
                    if ( ( (current_pos_.theta > (desired_pos_.theta + desired_pos_.theta*(-0.1))) && 
                           (current_pos_.theta < (desired_pos_.theta + desired_pos_.theta*0.1)) ) ) { 
                        
                        vel_data.linear.x = 0.0;
                        vel_data.angular.z = 0.0;
                
                        feedback->current_pos = current_pos_;  // action feedback data
                        goal_handle->publish_feedback(feedback);

                        flag = false;
                    }
                    else {  
                        vel_data.linear.x = 0.0;
                        vel_data.angular.z = 0.7;
                
                        feedback->current_pos = current_pos_;  // action feedback data
                        goal_handle->publish_feedback(feedback);
                    }
                }

                result->status = true;    
                goal_handle->succeed(result);
                RCLCPP_INFO(this->get_logger(), "Goal succeeded.");
                break;
            }

            loop_rate.sleep();
        }
    }
    //-------------------------------------------------------------------------------------------

    // odom sensor data acquisition and procesing 
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_pos_.x = msg->pose.pose.position.x;
        current_pos_.y = msg->pose.pose.position.y;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // convert radians to degrees
        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        current_pos_.theta = yaw * 180.0 / M_PI;
    }

    // publish velocities to move the robot 
    void velocityControlLoop() {
        cmd_vel_publisher_->publish(vel_data);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<GoToPoseActionServer> go_to_pose_node = std::make_shared<GoToPoseActionServer>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(go_to_pose_node);
    executor.spin();

    rclcpp::shutdown();

//   auto node = std::make_shared<GoToPoseActionServer>();
//   rclcpp::spin(node);
//   rclcpp::shutdown();


//    rclcpp::init(argc, argv);

//    std::shared_ptr<Patrol> robot_patrol_node = std::make_shared<Patrol>();

//    rclcpp::executors::MultiThreadedExecutor executor;
//    executor.add_node(robot_patrol_node);
//    executor.spin();

//    rclcpp::shutdown();
  
    return 0;
}