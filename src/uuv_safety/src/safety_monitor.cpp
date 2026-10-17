#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"

class SafetyMonitor : public rclcpp::Node
{
public:
    SafetyMonitor()
    : Node("safety_monitor")
    {
        // Subscribe to /dvl/altitude (seabed clearance, in metres).
        // Whenever a message arrives, ROS calls altitude_callback with it.
        altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/dvl/altitude", rclcpp::SensorDataQoS(),
            std::bind(&SafetyMonitor::altitude_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "safety_monitor started");
    }

private:
    void altitude_callback(const sensor_msgs::msg::Range & msg)
    {
        altitude_ = msg.range;
        RCLCPP_INFO(this->get_logger(), "altitude: %.2f m", altitude_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
    float altitude_ = 0.0f;   // latest altitude [m]
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}