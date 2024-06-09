#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <chrono>
#include <functional>
#include <limits>

using namespace std::chrono_literals;

// 'Patrol' class inherits from 'rclcpp::Node'
// making it a ROS2 node
class Patrol : public rclcpp::Node {
// scan sub and vel pub node implementation details
private:
    // laser scan subscriber members setup
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_subscriber_;
    std::string laser_topic = "scan";

    // main logic - 'scanCallback' method 
    void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr laser_data) {
        // total number of samples (720 readings)
        // const int size = laser_data->ranges.size();
        // RCLCPP_INFO(this->get_logger(), "%d", size);
        
        // define the ranges indices for -π/2 and π/2
        // index = (angle [°] + 180°) / (angle_increment [°])
        const int start_index = 180;
        const int end_index = 540;

        // initialize variables to track the maximum distance and its index
        /**
         * - 'max_distance' is initialized to negative inf to ensure any 
         *   valid distance will be larger
         *      'std::numeric_limits<float>::infinity()' returns the 
         *      positive infinity value of the given floating-point type
         * 
         * - 'max_index' is initialized to start_index as a placeholder
         */ 
        float max_distance = -std::numeric_limits<float>::infinity();
        int max_index = start_index;

        // read laser data between -pi/2 to pi/2
        // loop through the relevant part of the ranges array
        for (int i = start_index; i <= end_index; ++i) { // only front 180 degrees
            float range = laser_data->ranges[i];
            
            // ensuring range is between -inf and +inf
            if (range < std::numeric_limits<float>::infinity() && range > max_distance) {
                max_distance = range;
                max_index = i;
            }

            // calulate the angle corresponding to the max_index
            float max_angle = laser_data->angle_min + (max_index * laser_data->angle_increment);   

            RCLCPP_INFO(this->get_logger(), "Max distance: %f at angle: %f radians", max_distance, max_angle);
        }
    }

// laser sub and velocity pub node interface
// constructor
public:
    Patrol() : Node("patrol") {
        laser_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(this->laser_topic, 
                                                                                         10,
                                                                                         std::bind(&Patrol::laserCallback, 
                                                                                                   this, 
                                                                                                   std::placeholders::_1));
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Patrol>());
    rclcpp::shutdown();

    return 0;
}