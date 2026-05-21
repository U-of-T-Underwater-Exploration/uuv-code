#pragma once

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Dense"

namespace uuv_state_estimator {

/**
 * @brief Output struct representing the Full Vehicle State.
 * Pose includes Position and Orientation. 
 * Twist includes Linear and Angular Velocity.
 */
struct VehicleState {
    Eigen::Vector3d position;
    Eigen::Vector3d linear_velocity;
};

/**
 * @brief Linear Kalman Filter tailored for UUV position and velocity estimation.
 * 
 * Optimized for speed using Eigen fixed-size matrices (allocated on stack, fully unrolled).
 * Orientation (from Mahony) and Gyro are decoupled from the KF states for efficiency.
 * It tracks 6 states: [p_x, p_y, p_z, v_x, v_y, v_z]
 */
class KalmanFilter {
public:
    // Fixed-size matrix typedefs for zero-allocation performance 
    using Vector6d = Eigen::Matrix<double, 6, 1>;
    using Matrix6d = Eigen::Matrix<double, 6, 6>;
    using Matrix6x3d = Eigen::Matrix<double, 6, 3>;
    using Matrix3x6d = Eigen::Matrix<double, 3, 6>;
    using Matrix1x6d = Eigen::Matrix<double, 1, 6>;

private:
    Vector6d x_;         // State vector [px, py, pz, vx, vy, vz]^T
    Matrix6d P_;         // State covariance
    
    Matrix6d F_;         // State transition matrix
    Matrix6x3d B_;       // Control input matrix
    Matrix3x6d H_;       // Observation matrix
    Matrix1x6d H_baro_;  // Barometer observation matrix
    
    Matrix6d Q_;         // Process noise covariance
    Eigen::Matrix3d R_;  // Measurement noise covariance
    double R_baro_;      // Barometer measurement noise covariance

    Eigen::Vector3d gravity_; // Gravity vector in the inertial frame

public:
    KalmanFilter(double fp_, Eigen::Vector3f g_ref) {
        double dt = 1 / fp_

        // 1. Initialize state and covariance
        x_.setZero();
        P_.setIdentity();
        P_ *= 1.0; 
        
        // 2. Setup State Transition Matrix (F)
        F_.setIdentity();
        F_.topRightCorner<3, 3>() = Eigen::Matrix3d::Identity() * dt;
        
        // 3. Setup Control Input Matrix (B)
        B_.setZero();
        B_.topRows<3>() = Eigen::Matrix3d::Identity() * (0.5 * dt * dt);
        B_.bottomRows<3>() = Eigen::Matrix3d::Identity() * dt;
        
        // 4. Setup Observation Matrix (H) - assuming we measure Position only
        H_.setZero();
        H_.leftCols<3>() = Eigen::Matrix3d::Identity();
        
        H_baro_.setZero();
        H_baro_(0, 2) = 1.0; // Index 2 is p_z
        
        // 5. Initialize noises
        Q_.setIdentity();
        Q_ *= 0.01;      // Default process noise config
        
        R_.setIdentity();
        R_ *= 0.5;       // Default measurement noise config
        
        R_baro_ = 0.5;   // Default barometer noise config

        gravity_ = g_ref; // in NED already
    }

    void setBarometerNoise(double noise) { R_baro_ = noise; }
    void setProcessNoise(double noise) { Q_ = Matrix6d::Identity() * noise; }
    void setMeasurementNoise(double noise) { R_ = Eigen::Matrix3d::Identity() * noise; }

    /**
     * @brief Predict step: Propagates state forward using IMU acceleration
     * @param body_accel Linear acceleration from IMU (body frame)
     * @param orientation Orientation obtained from the Mahony filter
     */
    void predict(const Eigen::Vector3d& body_accel, const Eigen::Quaterniond& orientation) {
        // Rotate body acceleration back to the inertial frame 
        // and remove earth's gravity components.
        Eigen::Vector3d inertial_accel = (orientation * body_accel) - gravity_;
        
        // x_{k} = F * x_{k-1} + B * u_{k}
        x_ = F_ * x_ + B_ * inertial_accel;
        
        // P_{k} = F * P_{k-1} * F^T + Q
        P_ = F_ * P_ * F_.transpose() + Q_;
    }

    /**
     * @brief Update step: Integrates GPS / global position
     * @param gps_pos Position vector measured from GPS
     */
    void updateGPS(const Eigen::Vector3d& gps_pos) {
        // Innovation: y = z - H * x
        Eigen::Vector3d y = gps_pos - H_ * x_; 
        
        // Innovation covariance: S = H * P * H^T + R
        Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_; 
        
        // Kalman Gain: K = P * H^T * S^-1
        Matrix6x3d K = P_ * H_.transpose() * S.inverse(); 

        // Update state and covariance
        x_ = x_ + K * y;
        
        Matrix6d I = Matrix6d::Identity();
        P_ = (I - K * H_) * P_;
    }

    /**Update step: Integrates Barometer height/depth
     * @param baro_z Z-position measured from Barometer
     */
    void updateBarometer(double baro_z) {
        // Innovation: y = z - H * x
        double y = baro_z - x_[2]; 
        
        // Innovation covariance: S = H * P * H^T + R
        double S = (H_baro_ * P_ * H_baro_.transpose())(0, 0) + R_baro_; 
        
        // Kalman Gain: K = P * H^T * S^-1
        Vector6d K = P_ * H_baro_.transpose() / S; 

        // Update state and covariance
        x_ = x_ + K * y;
        
        Matrix6d I = Matrix6d::Identity();
        P_ = (I - K * H_baro_) * P_;
    }

    /**
     * @brief Packages the states together into a readable Pose/Twist representation
     */
    VehicleState getVehicleState(const Eigen::Quaterniond& orientation, const Eigen::Vector3d& gyro) const {
        VehicleState state;
        state.position = x_.head<3>();
        state.linear_velocity = x_.tail<3>();
        return state;
    }

    // Direct Getters
    Eigen::Vector3d getPosition() const { return x_.head<3>(); }
    Eigen::Vector3d getVelocity() const { return x_.tail<3>(); }
};

} // namespace uuv_state_estimator

