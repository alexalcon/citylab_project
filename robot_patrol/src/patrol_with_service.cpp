#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
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
 * This is node:
 *
 *      - Subscribes to laser data.
 *      - Calls a service ('direction_service') to pass this laser 
 *        data (i.e a service client).
 *      - Get the response of this service to puplish velocity data.
 * 
 * i.e. the service server provides the main obstacle avoidance logic 
 * (decision-making).
 */ 
class PatrolWithService : public rclcpp::Node {
// laser sub, velocity pub and service client node interface
// constructor
public:
    PatrolWithService() : Node("patrol_with_service_node") {
        callback_group_1= this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        callback_group_2= this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        
        // subscriber members initialization
        rclcpp::SubscriptionOptions options;
        options.callback_group = callback_group_1;
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            this->laser_topic, 
            10,
            std::bind(&PatrolWithService::laserCallback, this, _1),
            options);
        
        // service client initialization
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

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(this->vel_topic, 10);
        timer_ = this->create_wall_timer(
            100ms, 
            std::bind(&PatrolWithService::velocityControlLoop, this), 
            callback_group_2);
    }

// scan sub, vel pub and service client node implementation details
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

    void serviceResponse(rclcpp::Client<GetDirection>::SharedFuture fut) {
        auto status = fut.wait_for(std::chrono::seconds(1));
        if (status == std::future_status::ready) {
            auto response = fut.get();
            RCLCPP_INFO(this->get_logger(), "Service was called.");
            RCLCPP_INFO(this->get_logger(), "Recevied direction: %s", response->direction.c_str());
            vel_data = this->calculateVelocity(response->direction);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
        }
    }

    // method to calculte the right patrol navigation velocity 
    geometry_msgs::msg::Twist calculateVelocity(std::string direction) {
        auto velocity = geometry_msgs::msg::Twist();

        if (direction == "right") {
            velocity.linear.x = 0.1;
            velocity.angular.z = -0.5;   
        } 
        else if(direction == "front") {
            velocity.linear.x = 0.1;
            velocity.angular.z = 0.0;
        }
        else {
            velocity.linear.x = 0.1;
            velocity.angular.z = 0.5;   
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