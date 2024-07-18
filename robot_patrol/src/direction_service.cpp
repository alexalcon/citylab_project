#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "get_direction_interface/srv/get_direction.hpp"

using GetDirection = get_direction_interface::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

// 'DirectionService' class service server inherits from 'rclcpp::Node'
// making it a ROS2 (service server) node
/*
 * This service server node:
 *
 *      Performs the main obstacle avoidance logic (decision-making algorithm)
 *      for the robot to patrol autonomously in a "citylab" environment. 
 */
class DirectionService : public rclcpp::Node {
public:
    DirectionService() : Node("direction_service_node") {
        service_ = this->create_service<GetDirection>(
            "direction_service", 
            std::bind(&DirectionService::getDirection, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Direction service ready.");
  }

private:
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<get_direction_interface::srv::GetDirection>::SharedPtr service_;

    // main obstacle avoidance logic (decision-making algorithm)
    // service server callback function
    void getDirection(
        const std::shared_ptr<GetDirection::Request> request,
        const std::shared_ptr<GetDirection::Response> response) {
    
        // required data members to analyze the laser data
        float total_dist_sec_right = 0, total_dist_sec_front = 0, total_dist_sec_left = 0;
        int count_right = 0, count_front = 0, count_left = 0;
        
        // get the indices for the three sections
        /**
        * Formula to get the angle indices:
        *      index = (angle [°] + 180°) / (angle_increment [°])
        *      e.g.:
        *          For -90°: (-90° + 180) / (0.5°) = 180 
        *          For 90°: (90° + 180) / (0.5°) = 540 
        */ 
        int start_right_index = 180; // index for -90°
        int end_right_index = 300;   // index for -30°
        int end_front_index = 420;   // index for 30°
        int end_left_index = 540;    // index for 90°

        auto laser_data = request->laser_data;

        // logic to analyze/process the laser scan data
        // loop through the relevant part of the ranges array (front -π to π)
        for (int i = start_right_index; i <= end_left_index; ++i) {
            if (i >= start_right_index && i < end_right_index) {    // right section
                total_dist_sec_right += laser_data.ranges[i];
                ++count_right;
            }
            else if (i >= end_right_index && i < end_front_index) { // front section
                total_dist_sec_front += laser_data.ranges[i];
                ++count_front;
            }
            else if (i >= end_front_index && i < end_left_index) {  // left section
                total_dist_sec_left += laser_data.ranges[i];
                ++count_left;
            }
        }

        // average distances
        total_dist_sec_right /= count_right;
        total_dist_sec_front /= count_front;
        total_dist_sec_left /= count_left;

        // decide direction based on distances
        if (total_dist_sec_front > total_dist_sec_left && total_dist_sec_front > total_dist_sec_right) {
            response->direction = "forward";
        } else if (total_dist_sec_left > total_dist_sec_right) {
            response->direction = "left";
        } else {
            response->direction = "right";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DirectionService>());
    rclcpp::shutdown();
    
    return 0;
}