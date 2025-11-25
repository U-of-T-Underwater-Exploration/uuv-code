#pragma once

#include <cmath>

#include "rclcpp/rclcpp.hpp"

#include "Eigen/Dense"


namespace utux::state_estimator {

struct IMUCorrector{
    private: 
        const float CORRECTOR_DEFAULT_FS = 50.0f;

        // === Filter ===
        float fs_;   // [ Hz ] | Sampling freq of filter (same as state_estimator)
        float fc_;   // [ Hz ] | LPF cut-off freq. (fc < fs/2)
        float dt_;   // [ s ]  | Sampling period / time-step

        float b0_ = 0;  // 
        float b1_ = 0;  //  Filter coeff
        float a1_ = 0;  //


        // === Prev IMU values === 
        Eigen::Vector3f w_Prev_ = Eigen::Vector3f::Zero();        // Previous angular velocity

        Eigen::Vector3f alpha_prev_ = Eigen::Vector3f::Zero();       // Previous angular acceleration
        Eigen::Vector3f alpha_filterPrev_ = Eigen::Vector3f::Zero();   // Previous filtered angular acceleration

        // Translation
        Eigen::Vector3f r_baseToIMU_ = Eigen::Vector3f::Zero();    // Translation from body to imu frame

        // Internal Logic
        bool isInitialized_ = false;


        //  --------------- Helper Functions ---------------
        /**
         *  @brief Simple 1st order tustin (bilinear) digital low-pass filter
         */
        inline Eigen::Vector3f lpf(const Eigen::Vector3f &x, Eigen::Vector3f &xPrev, Eigen::Vector3f &yPrev) {
            Eigen::Vector3f y = b0_*x + b1_*xPrev + a1_*yPrev;
            xPrev = x;
            yPrev = y;
            return y;
        }

    public:
        //  --------------- Constructors ---------------
        /**
         *  @brief Default contructor too initially define corrector
         */
        IMUCorrector() : fs_(0.0f), fc_(0.0f), r_baseToIMU_(Eigen::Vector3f::Zero()) {
            isInitialized_ = false;
        }

        //  --------------- Initializers ---------------
        /**
         *  @brief Initializer for IMUCorrection module
         *  
         *  @param fs Sampling freq of the included angular acceleration lpf
         *  @param fc Angular acceleration lpf cut-off frequency
         *  @param r Translation vector from body to the imu
         *  
         *  @note Set `fc < fs/2`. For the filter to work correctly. Typically use fc ~ fs/10 or fs/5
         */
        void init(float fs, float fc, Eigen::Vector3f r) {
            // Set parameters
            fs_ = fs;
            fc_ = fc;
            r_baseToIMU_ = r;

            // Checks & corrections
            if(fs_ <= 0.0f) { fs_ = CORRECTOR_DEFAULT_FS; } // Ensure positive-definite smapling frequency
            dt_ = 1.0f/fs_;

            if(fc_ <= 0.0f) { fc_ += 1e-6; }                // Ensure pi*(fc_/fs_) <= pi/2
            if(fc_ >= (0.99/2.0f)*fs_) { fc_ += 1e-6; }     //    


            // Set filter coefficients
            const float K = std::tan(M_PI * (fc_/fs_)); // Can't have undefined behaviour
            b0_ = K/(K + 1.0f);
            b1_ = b0_;
            a1_ = (1.0f - K)/(K + 1.0f);

            // Set initial values
            w_Prev_.setZero();
            alpha_prev_.setZero();
            alpha_filterPrev_.setZero();

            isInitialized_ = true;
        }


        //  --------------- Member Functions ---------------
        /**
         *  @brief  Update/step the accelerometer correction + lpf
         * 
         *  @param acc IMU acceleration reading. NED + body_frame
         *  @param gyro IMU gyroscope reading. NED + body_frame
         * 
         *  @return Acceleration reading corected for offset and angular acceleration
         */
        inline Eigen::Vector3f update(const Eigen::Vector3f &acc, const Eigen::Vector3f &gyro) {
            if(!isInitialized_) {   // not-initialized fallback
                w_Prev_ = gyro;
                alpha_prev_.setZero();
                alpha_filterPrev_.setZero();
                return acc;
            }

            // Angular acceleration estimation (lpf + derivative)
            Eigen::Vector3f alpha = (gyro - w_Prev_)/dt_;
            w_Prev_ = gyro;

            Eigen::Vector3f alphaFilt = lpf(alpha, alpha_prev_, alpha_filterPrev_);
            alpha_prev_ = alpha;
            alpha_filterPrev_ = alphaFilt;

            Eigen::Vector3f tangential = alphaFilt.cross(r_baseToIMU_);
            Eigen::Vector3f centripetal = alphaFilt.cross(alphaFilt.cross(r_baseToIMU_));

            return (acc - tangential - centripetal);
        }
};

}   // utux::state_estimator