#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "diagnostic_msgs/msg/diagnostic_status.hpp"
#include "diagnostic_msgs/msg/key_value.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "uuv_dvl_driver/uuv_dvl_vel_parser.hpp"
#include "uuv_msgs/msg/kogger_dvl_vel.hpp"

class DVLDriver : public rclcpp::Node {
 public:
  DVLDriver() : Node("uuv_dvl_driver") {
    // Publisher 1: velocity
    twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("/dvl/twist", 10);

    // Publisher 2: altitude
    altitude_publisher_ = this->create_publisher<sensor_msgs::msg::Range>("/dvl/altitude", 10);

    // Publisher 3: diagnostics
    status_publisher_ =
        this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/dvl/status", 10);

    // Publisher 4: raw DVL measurements
    raw_publisher_ = this->create_publisher<uuv_msgs::msg::KoggerDvlVel>("/dvl/raw", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1), [this]() {
      auto stamp = this->now();

      // Create a simulated 68-byte DVL velocity payload
      std::vector<uint8_t> payload(68, 0);

      // TIMESTAMP = 1000 ms (little-endian uint32_t)
      payload[4] = 0xE8;
      payload[5] = 0x03;

      // VELOCITY_X = 1.0 m/s (little-endian IEEE 754 float)
      payload[16] = 0x00;
      payload[17] = 0x00;
      payload[18] = 0x80;
      payload[19] = 0x3F;

      // FLAGS = 257
      payload[0] = 0x01;
      payload[1] = 0x01;

      // DISTANCE_Z = 1.5 m
      payload[56] = 0x00;
      payload[57] = 0x00;
      payload[58] = 0xC0;
      payload[59] = 0x3F;

      // Decode and publish the simulated payload
      auto parsed = parse_dvl_vel(payload);

      // Publish the same decoded measurements to all topics

      publish_parsed_raw(stamp, parsed);
      publish_parsed_velocity(stamp, parsed);
      publish_parsed_altitude(stamp, parsed);
      publish_parsed_status(stamp, parsed);
    });

    RCLCPP_INFO(this->get_logger(), "DVL Driver started in simulated payload mode");
  }

 private:
  // Publisher 1: /dvl/twist

  void publish_parsed_velocity(const rclcpp::Time &stamp,
                               const std::map<std::string, double> &parsed) {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "dvl_link";

    // Velocities from your parser
    msg.twist.twist.linear.x = parsed.at("VELOCITY_X");
    msg.twist.twist.linear.y = parsed.at("VELOCITY_Y");
    msg.twist.twist.linear.z = parsed.at("VELOCITY_Z");

    // Variance = uncertainty squared
    double unc_x = parsed.at("UNCERTAINTY_X");
    double unc_y = parsed.at("UNCERTAINTY_Y");
    double unc_z = parsed.at("UNCERTAINTY_Z");

    msg.twist.covariance[0] = unc_x * unc_x;
    msg.twist.covariance[7] = unc_y * unc_y;
    msg.twist.covariance[14] = unc_z * unc_z;

    // Angular velocity is not measured by this message
    msg.twist.covariance[21] = 1000000.0;
    msg.twist.covariance[28] = 1000000.0;
    msg.twist.covariance[35] = 1000000.0;

    twist_publisher_->publish(msg);
  }

  // Publisher 2: /dvl/altitude
  void publish_parsed_altitude(const rclcpp::Time &stamp,
                               const std::map<std::string, double> &parsed) {
    sensor_msgs::msg::Range msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "dvl_link";

    msg.radiation_type = sensor_msgs::msg::Range::ULTRASOUND;

    // Kogger Micro DVL measurement range
    msg.min_range = 0.05;
    msg.max_range = 10.0;

    msg.range = static_cast<float>(parsed.at("DISTANCE_Z"));

    altitude_publisher_->publish(msg);
  }

  // Publisher 3: /dvl/status
  void publish_parsed_status(const rclcpp::Time &stamp,
                             const std::map<std::string, double> &parsed) {
    diagnostic_msgs::msg::DiagnosticArray msg;

    msg.header.stamp = stamp;

    diagnostic_msgs::msg::DiagnosticStatus status;

    status.name = "uuv_dvl_driver";
    status.hardware_id = "kogger_micro_dvl";

    status.level = diagnostic_msgs::msg::DiagnosticStatus::WARN;

    status.message = "Simulated payload - no DVL connected";

    // This is the simulated timer rate, not the sensor measurement (which is actually 1 MHz)
    add_status_value(status, "frame_rate_hz", "1.0");

    // These counters are not implemented yet.
    // Do not claim zero hardware errors.
    add_status_value(status, "checksum_errors", "not monitored");
    add_status_value(status, "unknown_id_count", "not monitored");

    // Actual FLAGS value decoded from the payload
    uint32_t flags = static_cast<uint32_t>(parsed.at("FLAGS"));

    add_status_value(status, "raw_flags", std::to_string(flags));

    msg.status.push_back(status);

    status_publisher_->publish(msg);
  }

  // Helper for adding diagnostic key-value pairs
  void add_status_value(diagnostic_msgs::msg::DiagnosticStatus &status, const std::string &key,
                        const std::string &value) {
    diagnostic_msgs::msg::KeyValue item;

    item.key = key;
    item.value = value;

    status.values.push_back(item);
  }

  // Publisher 4

  void publish_parsed_raw(const rclcpp::Time &stamp, const std::map<std::string, double> &parsed) {
    uuv_msgs::msg::KoggerDvlVel msg;

    msg.header.stamp = stamp;
    msg.header.frame_id = "dvl_link";

    // Packet header ID (change soon?)
    msg.id = 0x79;

    // Unsigned 32-bit fields
    msg.flags = static_cast<uint32_t>(parsed.at("FLAGS"));
    msg.device_timestamp_ms = static_cast<uint32_t>(parsed.at("TIMESTAMP"));

    // Timing information
    msg.delta_time = static_cast<float>(parsed.at("DELTA_TIME"));
    msg.latency = static_cast<float>(parsed.at("LATENCY"));

    // Velocities
    msg.vel_x = static_cast<float>(parsed.at("VELOCITY_X"));
    msg.vel_y = static_cast<float>(parsed.at("VELOCITY_Y"));
    msg.vel_z = static_cast<float>(parsed.at("VELOCITY_Z"));
    msg.vel_z1 = static_cast<float>(parsed.at("VELOCITY_Z1"));
    msg.vel_z2 = static_cast<float>(parsed.at("VELOCITY_Z2"));

    // Velocity uncertainties
    msg.unc_x = static_cast<float>(parsed.at("UNCERTAINTY_X"));
    msg.unc_y = static_cast<float>(parsed.at("UNCERTAINTY_Y"));
    msg.unc_z = static_cast<float>(parsed.at("UNCERTAINTY_Z"));
    msg.unc_z1 = static_cast<float>(parsed.at("UNCERTAINTY_Z1"));
    msg.unc_z2 = static_cast<float>(parsed.at("UNCERTAINTY_Z2"));

    // Distance measurements
    msg.dist_z = static_cast<float>(parsed.at("DISTANCE_Z"));
    msg.dist_z1 = static_cast<float>(parsed.at("DISTANCE_Z1"));
    msg.dist_z2 = static_cast<float>(parsed.at("DISTANCE_Z2"));

    raw_publisher_->publish(msg);
  }

  // Publisher declarations
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr twist_publisher_;

  rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr altitude_publisher_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr status_publisher_;

  rclcpp::Publisher<uuv_msgs::msg::KoggerDvlVel>::SharedPtr raw_publisher_;

  // Timer declaration
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DVLDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}