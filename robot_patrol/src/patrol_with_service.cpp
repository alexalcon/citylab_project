// FINAL FINISHED CODE FOR TASK1
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "get_direction_interface/srv/get_direction.hpp"

#include <iostream>
#include <string>
#include <cmath>
#include <chrono>
#include <functional>
#include <limits>

using GetDirection = get_direction_interface::srv::GetDirection;
using namespace std::chrono_literals;
using std::placeholders::_1;

// 'PatrolWithService' class inherits from 'rclcpp::Node'
// making it a ROS2 node
/*
 * This node:
 *
 *      - Subscribes to laser data.
 *      - Calls, continually, a service ('direction_service'), in the subscriber
 *        callback function, to pass this laser data (i.e a service client).
 *      - Get the response of this service to puplish velocity data.
 * 
 * i.e. the service server provides the main obstacle avoidance logic 
 * (decision-making).
 */ 
class PatrolWithService : public rclcpp::Node {
// laser scan sub, vel pub and service client node interface
// class node constructor
public:
    PatrolWithService() : Node("patrol_with_service_node") {
        callback_group_1 = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        callback_group_2 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // laser scan subscriber members initialization
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_1;
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->laser_topic, 
            10,
            std::bind(&PatrolWithService::laserCallback, this, _1),
            options);
        
        // service client initialization and call
        //---------------------------------------------------------------------
        client_ = this->create_client<GetDirection>("/direction_service");      

        // check if the service server is available or not 
        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                    "Interrupted while waiting for the service. Exiting...");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
                "Service not available, waiting again...");
        }
        //---------------------------------------------------------------------

        // velocity publisher members initialization
        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(this->vel_topic, 10);
        timer_ = this->create_wall_timer(
            100ms, // the publisher will send vel data each 0.1 [s]
            std::bind(&PatrolWithService::velocityControlLoop, this), 
            callback_group_2);
    }

// laser scan sub, vel pub and service client node implementation details
private:
    rclcpp::CallbackGroup::SharedPtr callback_group_1; // for laser subscriber
    rclcpp::CallbackGroup::SharedPtr callback_group_2; // for publisher timer

    // laser scan subscriber members setup
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::string laser_topic = "scan";

    // velocity publisher members setup
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::string vel_topic = "cmd_vel";
    
    // service client member setup
    rclcpp::Client<GetDirection>::SharedPtr client_;

    // additional node's data members 
    sensor_msgs::msg::LaserScan last_laser_;
    geometry_msgs::msg::Twist vel_data; 

    // logic to get needed laser scan data - 'laserCallback' method 
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        this->last_laser_ = *msg;
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = last_laser_;
        client_->async_send_request(request, std::bind(&PatrolWithService::serviceResponse, this, _1));   
    }

    // servide client response callback function
    void serviceResponse(rclcpp::Client<GetDirection>::SharedFuture fut) {
        auto status = fut.wait_for(std::chrono::seconds(1));
        if (status == std::future_status::ready) {
            auto response = fut.get();
            RCLCPP_INFO(this->get_logger(), "Service was called.");
            RCLCPP_INFO(this->get_logger(), "Recevied direction: %s", response->direction.c_str());
            // RCLCPP_INFO(this->get_logger(), "Recevied min distance: %f", response->min_distance);
            // RCLCPP_INFO(this->get_logger(), "Recevied min index: %d", response->min_index);
            vel_data = this->calculateVelocity(response->direction, response->min_distance, response->min_index);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
        }
    }

    // method to calculte the right patrol navigation velocity 
    geometry_msgs::msg::Twist calculateVelocity(std::string direction, float min_distance, int min_index) {
        auto velocity = geometry_msgs::msg::Twist();

        // if the min distance is closer than 35 cm to an obstacle in the robot's 
        // front then turn right or left according the max distance direction
        // 0.11999999731779099 is laser scan's range_min parameter value  
        if ( ( min_distance > 0.11999999731779099 ) && ( min_distance < (0.11999999731779099 + 0.11999999731779099*1.50) ) ) { // 0.70
            if ( ( min_index >= 270 && min_index < 360 ) ) { // counterclockwise rotation
                velocity.linear.x = 0.1;
                velocity.angular.z = 1.5; // 1.5
            }
            else if ( ( min_index > 360 && min_index <= 450 ) ) { // clockwise rotation
                velocity.linear.x = 0.1;
                velocity.angular.z = -1.5; // 1.5
            }
        } 
        else { // if there is no close obstacle then perform the server obstacle avoidance logic 
             if (direction == "right") {
                velocity.linear.x = 0.1;
                velocity.angular.z = 0.5;   
            } 
            else if(direction == "front") {
                velocity.linear.x = 0.1;
                velocity.angular.z = 0.0;
            }
            else {
                velocity.linear.x = 0.1;
                velocity.angular.z = -0.5;   
            }
        }

        return velocity;
    }

    // publish velocities to move the robot 
    void velocityControlLoop() {
        velocity_publisher_->publish(vel_data);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    
    // rclcpp::spin(std::make_shared<Patrol>());

    std::shared_ptr<PatrolWithService> robot_patrol_node = std::make_shared<PatrolWithService>();

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(robot_patrol_node);
    executor.spin();

    rclcpp::shutdown();

    return 0;
}