#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "get_direction_interface/srv/get_direction.hpp"

using GetDirection = get_direction_interface::srv::GetDirection;
using std::placeholders::_1;

// 'TestClient' class service client inherits from 'rclcpp::Node'
// making it a ROS2 (service client) node
/*
 * This service client (test) node:
 *
 *      Performs a continous service call, in a laser subscriber 
 *      callback function, in order to test the functionality of
 *      the service 'direction_service' from ./direction_service.cpp 
 */
class TestClient : public rclcpp::Node {
// laser scan subscriber and service client node interface
// class node constructor
public:
    TestClient() : Node("test_direction_service_node") {     
        // laser scan subscriber members initialization
        rclcpp::SubscriptionOptions sub_thread;
        sub_thread.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&TestClient::laserScanCallback, this, _1), 
            sub_thread);
        
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
    }


// laser scan subscriber and service client node implementation details
private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Client<GetDirection>::SharedPtr client_;

    void laserScanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request = std::make_shared<GetDirection::Request>();
        request->laser_data = *msg;
        client_->async_send_request(request, std::bind(&TestClient::serviceResponse, this, _1));
    }

    void serviceResponse(rclcpp::Client<GetDirection>::SharedFuture fut) {
        auto status = fut.wait_for(std::chrono::seconds(1));
        if (status == std::future_status::ready) {
            auto response = fut.get();
            RCLCPP_INFO(this->get_logger(), "Service was called.");
            RCLCPP_INFO(this->get_logger(), "Recevied direction: %s", response->direction.c_str());
            RCLCPP_INFO(this->get_logger(), "Recevied min distance: %f", response->min_distance);
            RCLCPP_INFO(this->get_logger(), "Recevied min index: %d", response->min_index);
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
        }
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestClient>());
    rclcpp::shutdown();

    return 0;
}