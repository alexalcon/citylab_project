#include "rclcpp/rclcpp.hpp"
#include "rclcpp/service.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "get_direction_interface/srv/get_direction.hpp"

#include <string>
#include <memory>

using GetDirection = get_direction_interface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

// 'DirectionService' class service server inherits from 'rclcpp::Node'
// making it a ROS2 node
class DirectionService : public rclcpp::Node {
public:
    DirectionService() : Node("direction_service_node") {
        srv_ = this->create_service<GetDirection>(
            "direction_service", 
             std::bind(&DirectionService::handleService, this, _1, _2));
    }

private:
    rclcpp::Service<GetDirection>::SharedPtr srv_;

    // service server 
    void handleService(
        const std::shared_ptr<GetDirection::Request> request,
        const std::shared_ptr<GetDirection::Response> response) {

        auto laser_data = request->laser_data;
        
        // get the indices for the three sections
        /**
        * Formula to get the angle indices
        *      index = (angle [°] + 180°) / (angle_increment [°])
        *      e.g.:
        *          For -90°: (-90° + 180) / (0.5°) = 180 
        *          For 90°: (90° + 180) / (0.5°) = 540 
        */ 
        int start_right_index = 180; // index for -90°
        int end_right_index = 300;   // index for -30°
        int end_front_index = 420;   // index for 30°
        int end_left_index = 540;    // index for 90°

        // logic to analyze/process the laser scan data
        // loop through the relevant part of the ranges array (front -π to π)
        for (int i = start_right_index; i <= end_left_index; ++i) {
            if (laser_data.ranges[i] >= start_right_index && laser_data.ranges[i] < end_right_index) {
                response->direction = "right section";
            }
            else if (laser_data.ranges[i] >= end_right_index && laser_data.ranges[i] < end_front_index) {
                response->direction = "front section";
            }
            else if (laser_data.ranges[i] >= end_front_index && laser_data.ranges[i] < end_left_index) {
                response->direction = "left section";
            }
        }

        RCLCPP_INFO(this->get_logger(), "section = %s", response->direction.c_str());
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();

    return 0;
}
