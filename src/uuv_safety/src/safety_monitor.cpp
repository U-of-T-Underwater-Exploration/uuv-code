#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "pwm_msg/msg/thrusters_pwm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "uuv_msgs/msg/kogger_dvl_vel.hpp"
#include "uuv_msgs/msg/safety_status.hpp"

class SafetyMonitor : public rclcpp::Node {
 public:
  SafetyMonitor() : Node("safety_monitor") {
    // ----------------------------------------------------------------
    // parameters Limits live in parameters so they can be changed without
    // recompiling. The numbers are PLACEHOLDERS: ask your lead for the real
    // ones.
    min_altitude_ = this->declare_parameter<double>("min_altitude", 0.5);  // m
    max_dvl_uncertainty_ =
        this->declare_parameter<double>("max_dvl_uncertainty", 0.1);  // m/s
    min_cell_voltage_ =
        this->declare_parameter<double>("min_cell_voltage", 3.2);  // V
    max_battery_temp_ =
        this->declare_parameter<double>("max_battery_temp", 60.0);  // deg C
    dvl_timeout_s_ =
        this->declare_parameter<double>("dvl_timeout_s", 1.0);  // s
    bms_timeout_s_ =
        this->declare_parameter<double>("bms_timeout_s", 3.0);  // s

    // Timers start counting from now, so a sensor that never speaks still
    // trips.
    last_dvl_time_ = this->now();
    last_bms_time_ = this->now();

    // ----------------------------------------------------------------
    // publisher
    status_pub_ = this->create_publisher<uuv_msgs::msg::SafetyStatus>(
        "/safety/status", 10);

    // ----------------------------------------------------------------
    // subscriptions Delivery settings for sensors: works with both reliable and
    // best-effort publishers.
    const auto qos = rclcpp::SensorDataQoS();

    altitude_sub_ = this->create_subscription<sensor_msgs::msg::Range>(
        "/dvl/altitude", qos,
        std::bind(&SafetyMonitor::altitude_callback, this,
                  std::placeholders::_1));

    dvl_raw_sub_ = this->create_subscription<uuv_msgs::msg::KoggerDvlVel>(
        "/dvl/raw", qos,
        std::bind(&SafetyMonitor::dvl_raw_callback, this,
                  std::placeholders::_1));

    bms_data_sub_ = this->create_subscription<sensor_msgs::msg::BatteryState>(
        "/bms/data", qos,
        std::bind(&SafetyMonitor::bms_data_callback, this,
                  std::placeholders::_1));

    bms_temp_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
        "/bms/temperature", qos,
        std::bind(&SafetyMonitor::bms_temp_callback, this,
                  std::placeholders::_1));

    pwm_sub_ = this->create_subscription<pwm_msg::msg::ThrustersPWM>(
        "/pwm/generated", qos,
        std::bind(&SafetyMonitor::pwm_callback, this, std::placeholders::_1));

    // ---------------------------------------------------------------- timer
    // Every 50 ms (20 Hz): run the checks and publish /safety/status.
    // A timer (not the callbacks) runs the checks so that a sensor that goes
    // silent is still noticed. A silent sensor never triggers its callback.
    timer_ = this->create_wall_timer(std::chrono::milliseconds(50),
                                     std::bind(&SafetyMonitor::on_timer, this));

    RCLCPP_INFO(this->get_logger(), "safety_monitor started");
  }

 private:
  // ------------------------------------------------------------------
  // callbacks Each callback only remembers the latest values. The checks happen
  // in on_timer().
  void altitude_callback(const sensor_msgs::msg::Range& msg) {
    altitude_ = msg.range;
  }

  void dvl_raw_callback(const uuv_msgs::msg::KoggerDvlVel& msg) {
    last_dvl_time_ = this->now();  // the DVL driver is alive
    dvl_uncertainty_ = std::max({msg.unc_x, msg.unc_y, msg.unc_z});
  }

  void bms_data_callback(const sensor_msgs::msg::BatteryState& msg) {
    last_bms_time_ = this->now();  // the BMS driver is alive
    power_health_ = msg.power_supply_health;
    if (!msg.cell_voltage.empty()) {
      lowest_cell_v_ =
          *std::min_element(msg.cell_voltage.begin(), msg.cell_voltage.end());
    }
  }

  void bms_temp_callback(const std_msgs::msg::Float32MultiArray& msg) {
    if (!msg.data.empty()) {
      hottest_temp_c_ = *std::max_element(msg.data.begin(), msg.data.end());
    }
  }

  void pwm_callback(const pwm_msg::msg::ThrustersPWM& msg) {
    int bad = 0;
    for (size_t i = 0; i < 8; i++) {  // channels 0-7 are the thrusters
      if (!msg.pwm_valid[i]) {
        bad++;
      }
    }
    pwm_bad_count_ = bad;
  }

  // ------------------------------------------------------------------ the
  // checks
  void on_timer() {
    std::vector<std::string> faults;    // things that must stop the thrusters
    std::vector<std::string> warnings;  // things worth reporting, but not fatal
    const rclcpp::Time now = this->now();

    // Sensor alive: has each driver sent anything recently?
    if ((now - last_dvl_time_).seconds() > dvl_timeout_s_) {
      faults.push_back("sensor_alive: /dvl/raw silent");
    }
    if ((now - last_bms_time_).seconds() > bms_timeout_s_) {
      faults.push_back("sensor_alive: /bms/data silent");
    }

    // Seabed: too close to the bottom. (NaN = no reading yet, so isfinite skips
    // it.)
    if (std::isfinite(altitude_) && altitude_ < min_altitude_) {
      faults.push_back("seabed: altitude below minimum");
    }

    // Battery: weak cell, hot battery, or the BMS reports a problem.
    if (std::isfinite(lowest_cell_v_) && lowest_cell_v_ < min_cell_voltage_) {
      faults.push_back("battery: cell voltage too low");
    }
    if (std::isfinite(hottest_temp_c_) && hottest_temp_c_ > max_battery_temp_) {
      faults.push_back("battery: temperature too high");
    }
    if (power_health_ >= 0 &&
        power_health_ != 1) {  // 1 = good, -1 = nothing received yet
      faults.push_back("battery: BMS reports a problem");
    }

    // PWM hardware: any thruster channel flagged invalid.
    if (pwm_bad_count_ > 0) {
      faults.push_back("pwm: thruster channel invalid");
    }

    // DVL quality: noisy DVL is only a warning.
    if (std::isfinite(dvl_uncertainty_) &&
        dvl_uncertainty_ > max_dvl_uncertainty_) {
      warnings.push_back("dvl_quality: uncertainty high");
    }

    // Combine into one state.
    uint8_t state = uuv_msgs::msg::SafetyStatus::OK;
    if (!faults.empty()) {
      state = uuv_msgs::msg::SafetyStatus::FAULT;
    } else if (!warnings.empty()) {
      state = uuv_msgs::msg::SafetyStatus::WARN;
    }

    // Publish /safety/status
    uuv_msgs::msg::SafetyStatus status;
    status.header.stamp = this->get_clock()->now();
    status.header.frame_id = "base_link";
    status.state = state;
    status.reasons = faults;
    status.reasons.insert(status.reasons.end(), warnings.begin(),
                          warnings.end());
    status_pub_->publish(status);

    // Log only when the state changes, so the terminal doesn't flood.
    if (state != last_state_) {
      RCLCPP_WARN(this->get_logger(), "safety state %d -> %d (%s)", last_state_,
                  state,
                  status.reasons.empty() ? "all clear"
                                         : status.reasons.front().c_str());
      last_state_ = state;
    }
  }

  // ------------------------------------------------------------------ members
  rclcpp::Publisher<uuv_msgs::msg::SafetyStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr altitude_sub_;
  rclcpp::Subscription<uuv_msgs::msg::KoggerDvlVel>::SharedPtr dvl_raw_sub_;
  rclcpp::Subscription<sensor_msgs::msg::BatteryState>::SharedPtr bms_data_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr
      bms_temp_sub_;
  rclcpp::Subscription<pwm_msg::msg::ThrustersPWM>::SharedPtr pwm_sub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Limits (from parameters)
  double min_altitude_;
  double max_dvl_uncertainty_;
  double min_cell_voltage_;
  double max_battery_temp_;
  double dvl_timeout_s_;
  double bms_timeout_s_;

  // Latest measurements. NaN / -1 mean "nothing received yet".
  const float kNaN = std::numeric_limits<float>::quiet_NaN();
  float altitude_ = kNaN;         // m
  float dvl_uncertainty_ = kNaN;  // m/s
  float lowest_cell_v_ = kNaN;    // V
  float hottest_temp_c_ = kNaN;   // deg C
  int power_health_ = -1;         // BatteryState health code (1 = good)
  int pwm_bad_count_ = 0;         // thruster channels flagged invalid

  rclcpp::Time last_dvl_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_bms_time_{0, 0, RCL_ROS_TIME};
  uint8_t last_state_ = uuv_msgs::msg::SafetyStatus::OK;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetyMonitor>());
  rclcpp::shutdown();
  return 0;
}