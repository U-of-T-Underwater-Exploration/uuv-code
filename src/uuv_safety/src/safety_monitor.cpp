#include <algorithm>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "pwm_msg/msg/thrusters_pwm.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "uuv_msgs/msg/kogger_dvl_vel.hpp"

class SafetyMonitor : public rclcpp::Node
{
public:
    SafetyMonitor()
    : Node("safety_monitor")
    {
        // Delivery settings for sensors: works with both reliable and best-effort publishers.
        const auto qos = rclcpp::SensorDataQoS();

        // /dvl/altitude: seabed clearance (m)
        altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
            "/dvl/altitude", qos,
            std::bind(&SafetyMonitor::altitude_callback, this, std::placeholders::_1));

        // /dvl/raw: proves the DVL driver is alive, and carries the DVL uncertainty (m/s)
        dvl_raw_sub_ = this->create_subscription<uuv_msgs::msg::KoggerDvlVel>(
            "/dvl/raw", qos,
            std::bind(&SafetyMonitor::dvl_raw_callback, this, std::placeholders::_1));

        // /pwm/generated: which PWM channels the hardware says are valid
        pwm_sub_ = this->create_subscription<pwm_msg::msg::ThrustersPWM>(
            "/pwm/generated", qos,
            std::bind(&SafetyMonitor::pwm_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "safety_monitor started");
    }

private:
    void altitude_callback(const sensor_msgs::msg::Range & msg)
    {
        altitude_ = msg.range;
        RCLCPP_INFO(this->get_logger(), "altitude: %.2f m", altitude_);
    }

    void dvl_raw_callback(const uuv_msgs::msg::KoggerDvlVel & msg)
    {
        // Keep the worst (largest) of the three uncertainties.
        dvl_uncertainty_ = std::max({msg.unc_x, msg.unc_y, msg.unc_z});
        RCLCPP_INFO(this->get_logger(), "DVL uncertainty: %.3f m/s", dvl_uncertainty_);
    }

    void pwm_callback(const pwm_msg::msg::ThrustersPWM & msg)
    {
        // Channels 0-7 are the thrusters. Count how many are flagged invalid.
        int bad = 0;
        for (size_t i = 0; i < 8; i++) {
            if (!msg.pwm_valid[i]) {
                bad++;
            }
        }
        pwm_bad_count_ = bad;
        RCLCPP_INFO(this->get_logger(), "PWM: %d of 8 thruster channels invalid", pwm_bad_count_);
    }

    rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
    rclcpp::Subscription<uuv_msgs::msg::KoggerDvlVel>::SharedPtr dvl_raw_sub_;
    rclcpp::Subscription<pwm_msg::msg::ThrustersPWM>::SharedPtr pwm_sub_;

    float altitude_ = 0.0f;         // latest altitude [m]
    float dvl_uncertainty_ = 0.0f;  // worst DVL uncertainty [m/s]
    int pwm_bad_count_ = 0;         // thruster channels flagged invalid
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SafetyMonitor>());
    rclcpp::shutdown();
    return 0;
}