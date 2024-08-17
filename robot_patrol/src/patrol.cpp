#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>

using namespace std::chrono_literals;

// 'Patrol' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class Patrol : public rclcpp::Node {
// scan sub and vel pub node implementation details
private:
    rclcpp::CallbackGroup::SharedPtr callback_group_1;
    rclcpp::CallbackGroup::SharedPtr callback_group_2;

    // laser scan subscriber members setup
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::string laser_topic = "scan";

    // velocity publisher members setup
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vel_topic = "cmd_vel";

    // additional node's data members 
    float direction_; // angular orientation (direction) to move the robot
    geometry_msgs::msg::Twist vel_data; 

    // logic to get needed laser scan data - 'laserCallback' method 
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
        // define the indices for -π/2 and π/2
        /**
         * Formula to get the angle indices
         *      index = (angle [°] + 180°) / (angle_increment [°])
         *      e.g.:
         *          For -90°: (-90° + 180) / (0.5°) = 180 
         *          For 90°: (90° + 180) / (0.5°) = 540 
         */ 
        const int start_index = 180;
        const int end_index = 540;

        // initialize variables to track the max and min distances and its indeces
        /**
         *  - 'max_distance' is initialized to negative inf to ensure any 
         *     valid distance will be larger.
         *          'std::numeric_limits<float>::infinity()' returns the 
         *          positive infinity value of the given floating-point type
         *  - 'max_index' is initialized to start_index as a placeholder.
         *  - 'min_distance' is initialized to positive inf to catch the min 
              distance. 
         *  - 'min_index' is initialized to end_index as a placeholder.
         */ 
        float max_distance = -std::numeric_limits<float>::infinity();
        float min_distance = std::numeric_limits<float>::infinity();
        int max_index = start_index;
        int min_index = end_index;

        // loop through the relevant part of the ranges array (front -π to π)
        for (int i = start_index; i <= end_index; ++i) {
            float current_range = laser_data->ranges[i];

            // non-exclusive conditionals to get the max and min distances and ranges indices
            //------------------------------------------------------------------------------------------------- 
            // ensuring range is between -inf and +inf, first, then between max_distance and +inf   
            // hence, getting the max distance and index
            if ( current_range < std::numeric_limits<float>::infinity() && current_range > max_distance ) {
                max_distance = current_range;
                max_index = i;
            }
            // reading from -15° (index 330) to 15° (index 390)
            // in order to get the min distance
            if ( i >= 330 && i <= 390 ) {
                if (current_range < std::numeric_limits<float>::infinity() && current_range < min_distance) {
                    min_distance = current_range;
                    min_index = i;
                }
            }
            //------------------------------------------------------------------------------------------------- 
        }

        // calulate the angle corresponding to the max_index
        this->direction_ = laser_data->angle_min + (max_index * laser_data->angle_increment);   
        // RCLCPP_INFO(this->get_logger(), "Max distance: %f [m] - Angle: %f [°] - Index: %d", max_distance, (this->direction_*180)/M_PI, max_index);
        // std::cout << max_distance << " - " << min_distance << std::endl;
        vel_data = this->calculateVelocity(max_distance, min_distance, direction_, min_index); 
    }

    // method to calculte the right patrol navigation velocity 
    geometry_msgs::msg::Twist calculateVelocity(float max_distance, float min_distance, float angle, int min_index) {
        auto velocity = geometry_msgs::msg::Twist();

        // if the min distance is closer than 35 cm to an obstacle in the robot's 
        // front then turn right or left according the max distance direction (angle)
        // 0.11999999731779099 is laser scan's range_min parameter value  
        if ( ( min_distance > 0.11999999731779099 ) && ( min_distance < (0.11999999731779099 + 0.11999999731779099*1.90) ) ) {
            if ( ( angle > 0 ) || ( min_index >= 330 && min_index < 360 ) ) { // counterclockwise rotation
                velocity.linear.x = 0;
                velocity.angular.z = 0.7;
                std::cout << "min_distace: " << min_distance << " " << velocity.angular.z << " " << angle << std::endl; 
            }
            else if ( ( angle < 0 ) || ( min_index > 360 && min_index <= 390 ) ) { // clockwise rotation
                velocity.linear.x = 0;
                velocity.angular.z = -0.7;
                std::cout << "min_distace: " << min_distance << " " << velocity.angular.z << " " << angle << std::endl; 
            }
        } 
        else { // if there is no close obstacle then move forward
            velocity.linear.x = 0.1;
            velocity.angular.z = 0.0;
            std::cout << "max distace: " << max_distance << std::endl; 
        }

        return velocity;
    }

    // publish velocities to move the robot 
    void velocityControlLoop() {
        velocity_publisher_->publish(vel_data);
    }

// laser sub and velocity pub node interface
// class node constructor
public:
    Patrol() : Node("patrol") {
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