#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"

#include "Eigen/Dense"

#include "uuv_state_estimator/imu_corrector.hpp"

using namespace std::chrono_literals;

namespace utux::state_estimator {

class StateEstimatorNode : public rclcpp::Node {
  private:
    // Parameters
    std::string frameId_;   // Coordinate frame of the odometry messages
    double fp_;             // [ Hz ] | Publishing rate  of the node

    // Tf
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    // Pub & Sub
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
    rclcpp::Subscription<sensor_msgs::msg::MagneticField>::SharedPtr sub_compass_;

    // State
    Eigen::Quaternionf q_bodyToWorld_; 
    Eigen::Vector3f a_body_;
    Eigen::Vector3f a_corrected_body_;
    Eigen::Vector3f w_body_;
    Eigen::Vector3f m_body_;

    // Transforms
    Eigen::Isometry3f T_baseToIMU_;
    Eigen::Isometry3f T_baseToCompass_;

    // Internal Tools
    IMUCorrector corrector_;
    
    void pub_odom_callback() {

        /**
         * TODO:
         * [ ] ENU --> NED
         * [ ] sensor_frame --> body_frame
         * [X] Correct a₆
         * [X] Make Normalized a₆ & B₆
         * [ ] Look-up g & m 
         * [ ] ori_ = MahonyFilter(a₆, ω₆, B₆, g, m)
         * [ ] [pose, twist] = KF(a₆, ω₆, ori, P)
         */

        //  Correct acceleration reading w/r to body
        a_corrected_body_ = corrector_.update(a_body_, w_body_);  
        
        // Normalized vectors
        Eigen::Vector3f a_corrNorm_body = a_corrected_body_.normalized();
        Eigen::Vector3f m_norm_body = m_body_.normalized();

        auto message = nav_msgs::msg::Odometry();

        /**
         * Set msg:
         *  Header
         *  child_frame
         *  Pose + Covariance
         *  Twist + Covaraiance
         */

        pub_odom_->publish(message);
      }

      void sub_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        a_body_ << msg->linear_acceleration.x,
                msg->linear_acceleration.y,
                msg->linear_acceleration.z;
        w_body_ << msg->angular_velocity.x,
                  msg->angular_velocity.y,
                  msg->angular_velocity.z;
        //RCLCPP_INFO(this->get_logger(), "imu callback is working%f%f%f", acc_[0], acc_[1], acc_[2]);
      }

      void sub_compass_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        m_body_ << msg->magnetic_field.x,
                msg->magnetic_field.y,
                msg->magnetic_field.z;
        //RCLCPP_INFO(this->get_logger(), "mag_callback is working%f%f%f", mag_[0], mag_[1], mag_[2]);
      }

  public:
    StateEstimatorNode() : Node("state_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)  {
      // Parameters
      this->declare_parameter("frame_id", "odom");
      this->declare_parameter("publish_rate", 50.0);
      this->declare_parameter("lpf_cutoff", 5.0);

      frameId_ = this->get_parameter("frame_id").as_string();
      fp_ = this->get_parameter("publish_rate").as_double();

      // Tf
      try {
        geometry_msgs::msg::TransformStamped tf_stamped;

        // Get IMU transformation
        tf_stamped = tf_buffer_.lookupTransform("imu_link", "base_link", tf2::TimePointZero, 500ms);
        T_baseToIMU_ = (tf2::transformToEigen(tf_stamped)).cast<float>();

        // Get Compass transformation
        tf_stamped = tf_buffer_.lookupTransform("compass_link", "base_link", tf2::TimePointZero, 500ms);
        T_baseToCompass_ = (tf2::transformToEigen(tf_stamped)).cast<float>();
      }
      catch (tf2::TransformException &ex){  // TF Fail report
        RCLCPP_WARN(this->get_logger(), "Couldn't get TF: [%s]", ex.what());
      }

      // Publisher & Subscribers
      std::chrono::duration<double, std::milli> publishPeriodChrono { 1000.0/fp_ };
      timer_ = this->create_wall_timer(publishPeriodChrono, std::bind(&StateEstimatorNode::pub_odom_callback, this));
      pub_odom_ = this->create_publisher<nav_msgs::msg::Odometry>("state_estimate", 10);
      sub_imu_ =  this->create_subscription<sensor_msgs::msg::Imu>(
        "imu/data", 10, std::bind(&StateEstimatorNode::sub_imu_callback, this, std::placeholders::_1));
      sub_compass_ = this->create_subscription<sensor_msgs::msg::MagneticField>(
        "compass/data", 10, std::bind(&StateEstimatorNode::sub_compass_callback, this, std::placeholders::_1));

      // Internal Tools
      corrector_.init(fp_, (float)(this->get_parameter("lpf_cutoff").as_double()), T_baseToIMU_.translation());

      // State Variables
      q_bodyToWorld_ = Eigen::Quaternionf(1.0, 0.0, 0.0, 0.0);
      a_body_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
      a_corrected_body_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
      w_body_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);
      m_body_ = Eigen::Vector3f(0.0f, 0.0f, 0.0f);

    }
};

}   // utux::state_estimator

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<utux::state_estimator::StateEstimatorNode>());
  rclcpp::shutdown();
  return 0;
}

