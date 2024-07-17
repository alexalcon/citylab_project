#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "get_direction_interface/srv/get_direction.hpp"

using GetDirection = get_direction_interface::srv::GetDirection;
using std::placeholders::_1;

class TestClient : public rclcpp::Node {
public:
    TestClient() : Node("test_direction_service_node") {     
        // subscriber members setup
        rclcpp::SubscriptionOptions sub_thread;
        sub_thread.callback_group = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 
            10, 
            std::bind(&TestClient::laserScancCallback, this, _1), sub_thread);
        
        // service client setup
        //------------------------------------------------------------------
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
        //------------------------------------------------------------------
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
    rclcpp::Client<GetDirection>::SharedPtr client_;

    void laserScancCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
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
        }
        else {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service.");
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestClient>());
    rclcpp::shutdown();

    return 0;
}