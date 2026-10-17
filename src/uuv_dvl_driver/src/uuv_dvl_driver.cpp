#include <chrono>
#include <memory>

#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

class DVLDriver : public rclcpp::Node {
 public:
  DVLDriver() : Node("uuv_dvl_driver") {
    twist_publisher_ =
        this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(
            "/dvl/twist", 10);

    timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                     [this]() { publish_dummy_velocity(); });

    RCLCPP_INFO(this->get_logger(), "DVL Driver started!");
  }

 private:
  void publish_dummy_velocity() {
    geometry_msgs::msg::TwistWithCovarianceStamped msg;

    msg.header.stamp = this->now();
    msg.header.frame_id = "dvl_link";

    msg.twist.twist.linear.x = 0.5;
    msg.twist.twist.linear.y = 0.2;
    msg.twist.twist.linear.z = -0.1;

    msg.twist.covariance[0] = 0.02 * 0.02;
    msg.twist.covariance[7] = 0.03 * 0.03;
    msg.twist.covariance[14] = 0.04 * 0.04;

    // Angular velocity is not measured in this demo.
    msg.twist.covariance[21] = 1000000.0;
    msg.twist.covariance[28] = 1000000.0;
    msg.twist.covariance[35] = 1000000.0;

    twist_publisher_->publish(msg);
  }

  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
      twist_publisher_;

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<DVLDriver>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
