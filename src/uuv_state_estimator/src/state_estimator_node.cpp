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
#include "uuv_state_estimator/mahony.hpp"
#include "uuv_state_estimator/kalman.hpp"

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

    // World references
    Eigen::Vector3f g_ref = Eigen::Vector3f(0, 0, 9.81f);
    Eigen::Vector3f m_ref = Eigen::Vector3f(1, 0, 0);

    // Transforms
    Eigen::Isometry3f T_IMUToBase;
    Eigen::Isometry3f T_CompassToBase_;

    // Internal Tools
    IMUCorrector corrector_;
    KalmanFilter kalman_filter_;
    Eigen::Vector3f *integral_error_state;
    
    void pub_odom_callback() {

        /**
         * TODO:
         * [X] ENU --> NED
         * [X] sensor_frame --> body_frame
         * [X] Correct a₆
         * [X] Make Normalized a₆ & B₆
         * [X] Look-up g & m 
         * [X] ori_ = MahonyFilter(a₆, ω₆, B₆, g, m)
         * [ ] [pose, twist] = KF(a₆, ω₆, ori, P)
         */

        //  Correct acceleration reading w/r to body
        a_corrected_body_ = corrector_.update(a_body_, w_body_);  
        
        // Mahony filter
        double dt = 1.0 / fp_; 
        MahonyResult mahony_result = mahonyFilterStep(a_corrected_body_, 
                                w_body_, 
                                m_body_, 
                                q_bodyToWorld_, 
                                dt, 
                                g_ref, 
                                m_ref,
                                {
                                  .max_iterations=1,
                                  .Kp=2.0f,
                                  .Ki=0.1f,
                                  .integral_error_state=integral_error_state});
        
        q_bodyToWorld_ = mahony_result.next_guess;
        // // Optionally log last error
        // RCLCPP_INFO(this->get_logger(), "MahonyError%f%f%f",mahony_result.error[0],mahony_result.error[1],mahony_result.error[2]);

        kalman_filter.predict(a_corrected_body_, q_bodyToWorld);
        // Use kalmanfilter.update*() to correct drift with gps or barometer
        /** TODO: Process and measurement variance NOT implemented */ 
        VehicleState state = kalman_filer.getVehicleState();

        auto message = nav_msgs::msg::Odometry();

        /**
         * Set msg:
         *  Header
         *  child_frame
         *  Pose + Covariance
         *  Twist + Covaraiance
         */

        message.header.stamp = this->get_clock()->now();
        message.header.frame_id = "odom";
        message.child_frame_id = "base_link";

        message.pose.pose.position = state.position;
        message.pose.pose.orientation = q_bodyToWorld_;

        message.twist.twist.linear = state.linear_velocity;
        message.twist.twist.angular = w_body_;

        pub_odom_->publish(message);
      }

      void sub_imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        Eigen::Vector3f a_sensor_;
        Eigen::Vector3f w_sensor_;

        a_sensor_ << msg->linear_acceleration.y,
                msg->linear_acceleration.x,
                -msg->linear_acceleration.z;
        w_sensor_ << msg->angular_velocity.y,
                  msg->angular_velocity.x,
                  -msg->angular_velocity.z;

        // frame transformation
        a_body_ = T_IMUToBase.linear() * a_sensor_;
        w_body_ = T_IMUToBase.linear() * w_sensor_;
        //RCLCPP_INFO(this->get_logger(), "imu callback is working%f%f%f", acc_[0], acc_[1], acc_[2]);
      }

      void sub_compass_callback(const sensor_msgs::msg::MagneticField::SharedPtr msg) {
        Eigen::Vector3f m_sensor_;

        m_sensor_ << msg->magnetic_field.y,
                msg->magnetic_field.x,
                -msg->magnetic_field.z;
        
        // frame transformation
        m_body_ = T_CompassToBase_.linear() * m_sensor_;
        //RCLCPP_INFO(this->get_logger(), "mag_callback is working%f%f%f", mag_[0], mag_[1], mag_[2]);
      }

  public:
    StateEstimatorNode() : Node("state_estimator"), tf_buffer_(this->get_clock()), tf_listener_(tf_buffer_)  {
      // Parameters
      this->declare_parameter("frame_id", "odom");
      this->declare_parameter("publish_rate", 50.0);
      this->declare_parameter("lpf_cutoff", 5.0);

      // in NED frame, default Toronto, Canada
      this->declare_parameter("g_ref", 9.81); // m/s^2
      this->declare_parameter("magnetic_ref_hor", -16.7) // microT
      this->declare_parameter("magnetic_ref_ver", 52.5) // microT

      frameId_ = this->get_parameter("frame_id").as_string();
      fp_ = this->get_parameter("publish_rate").as_double();
      g_ref[2] = this->get_parameter("g_ref").as_double();
      mag_ref[0] = this->get_parameter("magnetic_ref_ver").as_double();
      mag_ref[1] = this->get_parameter("magnetic_ref_hor").as_double();

      // Tf
      try {
        geometry_msgs::msg::TransformStamped tf_stamped;

        // Get IMU transformation
        tf_stamped = tf_buffer_.lookupTransform("base_link", "imu_link", tf2::TimePointZero, 500ms);
        T_IMUToBase_ = (tf2::transformToEigen(tf_stamped)).cast<float>();

        // Get Compass transformation
        tf_stamped = tf_buffer_.lookupTransform("base_link", "compass_link", tf2::TimePointZero, 500ms);
        T_CompassToBase_ = (tf2::transformToEigen(tf_stamped)).cast<float>();
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
      corrector_.init(fp_, (float)(this->get_parameter("lpf_cutoff").as_double()), T_IMUToBase.translation());
      kalman_filter_.init(fp_, g_ref);

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

