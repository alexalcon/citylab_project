#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

// 'Patrol' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class Patrol : public rclcpp::Node {
// laser scan sub and vel pub node implementation details
private:
    // node data members setup
    //-------------------------------------------------------------------------------
    rclcpp::CallbackGroup::SharedPtr callback_group_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_2;

    // laser scan subscriber members setup
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::string laser_topic = "scan";
    
    // vel publisher members setup
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    std::string vel_topic = "cmd_vel";
    rclcpp::TimerBase::SharedPtr timer_;

    // angular orientation (direction) to move the robot
    float direction_ = 0.0;
    //-------------------------------------------------------------------------------

    // main logic - 'scanCallback' method - find the safest direction to move
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
        // total number of samples (720 readings)
        // const int size = laser_data->ranges.size();
        // RCLCPP_INFO(this->get_logger(), "%d", size);

        float max_distance = laser_data->range_min;
        int max_index = 0;
        bool flag = true;
        const float safest_distance = 0.5;

        // read all the laser data indices - not just the ones from -pi/2 to pi/2  
        for (int i = 0; i < 720; i++) {
            // ensure a safest distance index to calculate the safest angle 
            if (laser_data->ranges[719-i] > safest_distance && flag) {
                // get the maximum distance to move 
                if(laser_data->ranges[719-i] > max_distance && laser_data->ranges[719-i] != INFINITY ) {
                    max_distance = laser_data->ranges[719-i];
                    max_index = 719-i;
                }
            }
            // an unsafe distance index is gotten 
            else if (laser_data->ranges[719-i] <= safest_distance || !flag) {
                flag = false;
                if (max_index < 360 && (719-i) < 360 
                                    && max_distance < laser_data->ranges[719-i] 
                                    && laser_data->ranges[719-i] != INFINITY) { // left obstacle
                    max_distance = laser_data->ranges[719-i];
                    max_index = 719-i;
                } else if (max_index > 359 && (719-i) > 359 
                                           && max_distance < laser_data->ranges[719-i] 
                                           && laser_data->ranges[719-i] != INFINITY) { // right obstacle
                    max_distance = laser_data->ranges[719-i];
                    max_index = 719-i;
                }
            }
        }

        // calculate the safest direction's angle
        // ----------------------------------------
        // this->direction_ = (max_index/719.0)*M_PI - M_PI/2; // if index [0] -> -90 and index [719] -> 90  
        this->direction_ = M_PI/2 - (max_index/719.0)*M_PI; // if index index [0] -> 90 and [719] -> -90   
        // RCLCPP_INFO(this->get_logger(), "max_distance: %f [m] - max_index: %d - angle: %f [rad]", max_distance, max_index, this->direction_);
        RCLCPP_INFO(this->get_logger(), "max_distance: %f [m] - angle: %f [rad] - ang_vel: %f [rad/s]", max_distance, this->direction_, this->direction_ / 2);
    }

    // publish velocities to move the robot 
    void velocityControlLoop() {
        auto vel_data = geometry_msgs::msg::Twist();
        vel_data.linear.x = 0.1;                   // constant linear velocity in x
        vel_data.angular.z = this->direction_ / 2; // constant angular velocity in z

        velocity_publisher_->publish(vel_data);
    }

// laser scan sub and velocity pub node interface
// constructor
public:
    Patrol() : Node("robot_patrol") {
        callback_group_1= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        callback_group_2= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_1;
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->laser_topic, 
                                                                                         10,
                                                                                         std::bind(&Patrol::laserCallback, 
                                                                                                   this, 
                                                                                                   std::placeholders::_1),
                                                                                         options);

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(this->vel_topic, 10);
        timer_ = this->create_wall_timer(100ms, std::bind(&Patrol::velocityControlLoop, this), callback_group_2);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // rclcpp::spin(std::make_shared<Patrol>());
    
    std::shared_ptr<Patrol> robot_patrol_node = std::make_shared<Patrol>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_patrol_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}