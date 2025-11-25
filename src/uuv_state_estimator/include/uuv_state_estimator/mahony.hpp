// mahony(acc,angval,mag,curr_guess,ref_g,ref_m,max_iteration) -> next_guess, quaternions in the form 
// all the variables are in eigen --vector3d (look up the documentation)
// current guess is an eigen :: quaterniond 
// iteration is an int
// return next_guess as an eigen :: quaternion using mahony filter and error as a --vector3d


#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

struct MahonyResult
{
  Eigen::Quaternionf next_guess;  // updated orientation estimate after going throuch k iterations (body -> world)
  Eigen::Vector3f error;          // last corrective error, acc error plus magento error (body frame)
};

// Helper: rotate a world-frame vector into body frame using q (body->world)
inline Eigen::Vector3f worldToBody(const Eigen::Quaternionf &q_body_to_world,
                                   const Eigen::Vector3f &v_world)
{
  // body = q* v * q.conjugate  if q: body->world
  // => world->body uses q.conjugate()
  return q_body_to_world.conjugate() * v_world; //gives v_world expressed in terms of the body frame
}

inline MahonyResult mahonyFilterStep(
    const Eigen::Vector3f &accel_body,               // measured accel (body frame)
    const Eigen::Vector3f &gyro_body,                // measured angular vel (rad/s, body frame)
    const Eigen::Vector3f &mag_body,                 // measured mag (body frame)
    const Eigen::Quaternionf &q_current_body_world,  // current orientation estimate (body->world), AKA current_guess
    float dt,                                        // difference in seconds between last mahony call and current call
    const Eigen::Vector3f &g_ref_world,              // reference gravity direction (world frame)
    const Eigen::Vector3f &mag_ref_world,            // reference mag field (world frame)
    int max_iterations,                              // number of steps for mahony filter to use
    float Kp = 2.0f,                                 // proportional gain
    float Ki = 0.1f,                                 // integral gain
    Eigen::Vector3f *integral_error_state = nullptr  // persistent I-term state (body frame), create the vector in the ROS node so it can persist between each mahony estimation call
)
{
  MahonyResult result; // stores the result for our next guess and error after running the filter
  Eigen::Quaternionf q = q_current_body_world; // local copy of our current guess

  // Make sure we do at least one iteration, fail safe for our call
  if (max_iterations < 1) {
    max_iterations = 1;
  }

  // make sure that dt is not too big or too large, 
  if (dt <= 0 || dt > 0.1f){
    dt = 0.01f;
  }

  // lets us integrate the quaternion in multiple small steps
  float dt_step = dt / static_cast<float>(max_iterations);

  // carry over error state from the past mahony call
  Eigen::Vector3f integral_error = Eigen::Vector3f::Zero();
  if (integral_error_state) {
    integral_error = *integral_error_state;
  }

  Eigen::Vector3f last_error = Eigen::Vector3f::Zero();

  for (int i = 0; i < max_iterations; ++i) {
    // Normalize acc and mag, aprox dir of gravity and dir of mag in body coords
    Eigen::Vector3f a = accel_body.normalized();
    Eigen::Vector3f m = mag_body.normalized();

    // normalize world g and world mag from world to body frame
    Eigen::Vector3f g_body_est = worldToBody(q, g_ref_world).normalized();
    Eigen::Vector3f mag_body_est = worldToBody(q, mag_ref_world).normalized();

    // calculating mahony error, there are no lambda terms, i dont quite understand how to get them lambds is the weight term
    Eigen::Vector3f e_g = a.cross(g_body_est);
    Eigen::Vector3f e_m = m.cross(mag_body_est);
    Eigen::Vector3f e = e_g + e_m;

    last_error = e;

    // update the integral term, we estimate the continuous integral of our current e
    if (Ki > 0.0f) {
      integral_error += e * dt_step;
    }

    // our current mahony PI term for this iteration - corrects our estimate towards our references
    Eigen::Vector3f gyro_corrected = gyro_body + Kp * e + Ki * integral_error;

    // integrate quaternion ( q_dot = 0.5 * q ⊗ [0, gyro_corrected] )
    // return our current q^t+1 estimate
    Eigen::Quaternionf omega_quat(0.0f,
                                  gyro_corrected.x(),
                                  gyro_corrected.y(),
                                  gyro_corrected.z());

    Eigen::Quaternionf q_dot = q * omega_quat;
    q.coeffs() += 0.5f * dt_step * q_dot.coeffs(); 

    // 7) Normalize quaternion to avoid drift
    q.normalize();
  }

  if (integral_error_state) {
    *integral_error_state = integral_error;
  }

  result.next_guess = q;
  result.error = last_error;
  return result;
}