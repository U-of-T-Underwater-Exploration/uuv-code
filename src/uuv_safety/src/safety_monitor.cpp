#include "rclcpp/rclcpp.hpp"

// A node is a class that inherits from rclcpp::Node.
class SafetyMonitor : public rclcpp::Node
{
public:
    SafetyMonitor()
    : Node("safety_monitor")   // the name this node will have in ROS
    {
        RCLCPP_INFO(this->get_logger(), "safety_monitor started");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);                          // start ROS in this program
    rclcpp::spin(std::make_shared<SafetyMonitor>());   // create the node and keep it alive
    rclcpp::shutdown();                                // clean up when it stops
    return 0;
}