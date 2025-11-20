#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include "Eigen/Dense"

using namespace std::chrono_literals;

class StateEstimatorNode : public rclcpp::Node {
  // Parameters
  std::string frameId_;   // [ ]
  double publishRate_;    // [ Hz ]

  // Pub & Sub
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisherOdom_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr subImu_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr subMag;

  // State Variables
  Eigen::Quaterniond ori_; 
  Eigen::Vector3d acc_;
  Eigen::Vector3d angVel_;
  Eigen::Vector3d mag_;

  private:
    void timer_callback() {

      /**
       * TODO:
       * - Create normalized a₆, ω₆, B₆
       * - Look-up g & m 
       * - ori = MahonyFilter(a₆, ω₆, B₆, g, m)
       * - [pose, twist] = KF(a₆, ori, p)
       */

      auto message = nav_msgs::msg::Odometry();

      /**
       * Set msg:
       *  Header
       *  child_frame
       *  Pose + Covariance
       *  Twist + Covaraiance
       */

      publisherOdom_->publish(message);
    }

  public:
    StateEstimatorNode() : Node("state_estimator") {
      // Parameters
      this->declare_parameter("frame_id", "odom");
      this->declare_parameter("publish_rate", 50.0);

      frameId_ = this->get_parameter("frame_id").as_string();
      publishRate_ = this->get_parameter("publish_rate").as_double();


      std::chrono::duration<double, std::milli> publishPeriodChrono { 1000.0/publishRate_ };

      // Publisher & Subscribers
      timer_ = this->create_wall_timer(publishPeriodChrono, std::bind(&StateEstimatorNode::timer_callback, this));
      publisherOdom_ = this->create_publisher<nav_msgs::msg::Odometry>("state_estimate", 10);

      // State Variables
      ori_ = Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0);
    }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}
