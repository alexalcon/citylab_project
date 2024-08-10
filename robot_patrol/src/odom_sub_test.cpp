#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"

#include <memory>
#include <unistd.h>

// for the parameters
using namespace std::chrono_literals; // templates of seconds or ms 
using std::placeholders::_1;

// 'OdometrySubscriberTest' class inherits from 'rclcpp::Node'
// making it a ROS2 (odometry subscriber) node
class OdometrySubscriberTest : public rclcpp::Node {
// odometry subscriber node interface
// class node constructor
public:
    OdometrySubscriberTest() : Node("odom_sub_test_node") {
        // odom sub members initialization
        rclcpp::SubscriptionOptions odom_sub_options;
        odom_sub_options.callback_group =  this->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);    
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&OdometrySubscriberTest::odomSubscriberCallback, this, _1),
            odom_sub_options);
    }

// odometry subscriber and service client node implementation details
private:
    rclcpp::CallbackGroup::SharedPtr odom_sub_callbackgroup_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;    

    void odomSubscriberCallback(const nav_msgs::msg::Odometry::SharedPtr odom_data) {
        // extract the position
        RCLCPP_INFO(this->get_logger(), 
            "Position -> x: %f y: %f z: %f", 
            odom_data->pose.pose.position.x,
            odom_data->pose.pose.position.y,
            odom_data->pose.pose.position.z);

        // extract the orientation quaternion
        double roll, pitch, yaw;
        tf2::Quaternion q(
            odom_data->pose.pose.orientation.x,
            odom_data->pose.pose.orientation.y,
            odom_data->pose.pose.orientation.z,
            odom_data->pose.pose.orientation.w);

        // convert quaternion to Euler angles (in radians)
        tf2::Matrix3x3 m(q);
        m.getRPY(roll, pitch, yaw);

        // convert radians to degrees
        double roll_deg = roll * 180.0 / M_PI;
        double pitch_deg = pitch * 180.0 / M_PI;
        double yaw_deg = yaw * 180.0 / M_PI;

        // output the Euler angles in degrees
        RCLCPP_INFO(this->get_logger(), 
            "Orientation-> yaw: %f °", yaw_deg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    std::shared_ptr<OdometrySubscriberTest> odom_sub_test_node = 
        std::make_shared<OdometrySubscriberTest>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(odom_sub_test_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}