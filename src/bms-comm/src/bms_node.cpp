#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

#include "bms_parser.hpp"

using namespace std::chrono_literals;

class BMSNode : public rclcpp::Node
{
public:
  BMSNode()
  : Node("bms_node"), serial_port_(-1)
  {
	// Parameters
	this->declare_parameter("serial_port", "/dev/ttyS0");
	this->declare_parameter("baud_rate", 115200);
	this->declare_parameter("publish_frequency", 2.0);

	std::string port = this->get_parameter("serial_port").as_string();
	int baud = this->get_parameter("baud_rate").as_int();
	double freq = this->get_parameter("publish_frequency").as_double();

	// QoS: Reliable, depth 10
	auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();

	// Publishers
	battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/bms/data", qos);
	temp_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/bms/temperature", qos);

	// Initialize serial
	if (!init_serial(port, baud)) {
	  RCLCPP_ERROR(this->get_logger(), "Failed to initialize serial port %s", port.c_str());
	}

	// Timer
	timer_ = this->create_wall_timer(
	  std::chrono::duration<double>(1.0 / freq),
	  std::bind(&BMSNode::timer_callback, this));
  }

  ~BMSNode() {
	if (serial_port_ >= 0) {
	  close(serial_port_);
	}
  }

private:
  bool init_serial(const std::string& port, int baud) {
	serial_port_ = open(port.c_str(), O_RDWR);
	if (serial_port_ < 0) return false;

	struct termios tty;
	if (tcgetattr(serial_port_, &tty) != 0) return false;

	speed_t speed;
	switch(baud) {
	  case 9600: speed = B9600; break;
	  case 115200: speed = B115200; break;
	  default: speed = B115200; break;
	}

	cfsetispeed(&tty, speed);
	cfsetospeed(&tty, speed);

	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;
	tty.c_cflag |= CREAD | CLOCAL;
	tty.c_lflag &= ~ICANON;
	tty.c_lflag &= ~ECHO;
	tty.c_lflag &= ~ECHOE;
	tty.c_lflag &= ~ECHONL;
	tty.c_lflag &= ~ISIG;
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
	tty.c_oflag &= ~OPOST;
	tty.c_oflag &= ~ONLCR;
	tty.c_cc[VTIME] = 10; // 1 second timeout
	tty.c_cc[VMIN] = 0;

	if (tcsetattr(serial_port_, TCSANOW, &tty) != 0) return false;
	return true;
  }

  void timer_callback() {
	if (serial_port_ < 0) {
	  RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Serial port not open");
	  return;
	}

	// Send Request: Full Read Buffer
	uint8_t cmd[] = {0x4E, 0x57, 0x00, 0x13, 0x00, 0x00, 0x00, 0x00, 
					 0x06, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
					 0x68, 0x00, 0x00, 0x01, 0x29};
	
	if (write(serial_port_, cmd, sizeof(cmd)) != sizeof(cmd)) {
	  RCLCPP_ERROR(this->get_logger(), "Failed to write to serial port");
	  return;
	}

	// Read Response
	std::vector<uint8_t> buffer(512);
	int n = read(serial_port_, buffer.data(), buffer.size());
	if (n <= 0) {
	  RCLCPP_ERROR(this->get_logger(), "Failed to read from serial port (n=%d)", n);
	  return;
	}

	BMSData data;
	auto now = this->get_clock()->now();

	if (parseBMSFrame(buffer.data(), n, data)) {
	  publish_data(data, now);
	} else {
	  RCLCPP_ERROR(this->get_logger(), "Failed to parse BMS frame");
	}
  }

  void publish_data(const BMSData& data, const rclcpp::Time& stamp) {
	// BatteryState
	sensor_msgs::msg::BatteryState msg;
	msg.header.stamp = stamp;
	msg.header.frame_id = "bms_link";
	msg.voltage = data.voltage;
	msg.current = data.current;
	msg.percentage = data.capacityPercentage / 100.0;
	msg.temperature = data.temperatureMOSFET;
	msg.charge = data.capacityAh;
	msg.capacity = 16.0; // Total 16Ah
	msg.design_capacity = 16.0;
	msg.cell_voltage = std::vector<float>(data.cellVoltages.begin(), data.cellVoltages.end());
	msg.power_supply_status = data.isCharging ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING :
							 (data.isDischarging ? sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING :
							  sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING);
	
	battery_pub_->publish(msg);

	// Temperatures as Float32MultiArray
	std_msgs::msg::Float32MultiArray temp_msg;
	temp_msg.data = {
		static_cast<float>(data.temperatureMOSFET),
		static_cast<float>(data.temperatureProbe1),
		static_cast<float>(data.temperatureProbe2)
	};
	temp_pub_->publish(temp_msg);
  }

  int serial_port_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr temp_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BMSNode>());
  rclcpp::shutdown();
  return 0;
}
