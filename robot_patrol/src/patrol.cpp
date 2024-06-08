#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <string>
#include <chrono>
#include <functional>

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
        // initial members setup to catch the largest range value from the laser 
        //-----------------------------------------------------------------------
        // total number of samples (720 readings)
        const int size = laser_data->ranges.size();
        RCLCPP_INFO(this->get_logger(), "%d", size);
        
        // int min_index = -1;
        
        // // returns the positive infinity value of the given floating-point type
        // float min_range =  std::numeric_limits<float>::infinity();  
        //-----------------------------------------------------------------------

    // for(int i = 0 ; size / 2 ; i++) { // only front 180 degrees
    //         if () {
    //         }
    //     }
        
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