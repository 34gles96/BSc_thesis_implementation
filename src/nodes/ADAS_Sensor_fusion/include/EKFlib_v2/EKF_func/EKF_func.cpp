//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_func.cpp
//
// Code generation for function 'EKF_func'
//

// Include files
#include "EKF_func.h"
#include "EKF_func_data.h"
#include "EKF_func_initialize.h"
#include "EKF_func_types.h"
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cmath>
#include <cstring>

// Variable Definitions
static double prev_timestamp;

static double T_enu_ecef[16];

static double R_body_sensor[9];

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

void EKF_func(const struct0_T *IMU, const struct2_T *GNSS,
              const struct3_T *Vehicle, struct4_T *state)
{
  static const signed char b_I[100]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const signed char iv[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
  double H[60];
  double G_gyro[30];
  double R_position1[9];
  double R_position2[9];
  double delta_q[4];
  double omega[3];
  if (!isInitialized_vehicle_ekf) {
    EKF_func_initialize();
  }
  //
  //  Main EKF function.
  //
  //  state.p                  3x1 ENU position
  //  state.q                  4x1 Body -> ENU quaternion
  //  state.bg                 3x1 gyro bias
  //  state.P                  covariance matrix
  //
  //  IMU.timestamp  IMU measurement timestamp [ms]
  //  GNSS.timestamp GNSS measurement timestamp [ms]
  //
  //  prev_timestamp is persistent and stored in [ms].
  //  Time differences used by the prediction model are converted to [s].
  //  state.has_prev_imu_time  true after first IMU message
  //
  //  hom trasnfromation matrix
  //  initialization
  //  =========================================================
  //   WAITING FOR FIRST VALID GNSS
  //  ==========================================================
  //  in the first gnss data transmission: init of ENU origin
  if ((!(prev_timestamp == -1.0)) || GNSS->valid) {
    //  =========================================================
    //   INITIALIZE FIXED ENU FRAME, construction transfromation fromIMU2body
    //  ==========================================================
    if (prev_timestamp < 0.0) {
      double N;
      double a;
      double omega_norm;
      double theta;
      double yaw_den;
      double yaw_num;
      int k;
      //  Constant values during transformation from LLA to ECEF
      //  Earth flattening
      omega_norm = std::sin(GNSS->lla1[0]);
      N = 6.378137E+6 /
          std::sqrt(1.0 - 0.0066943799901413165 * (omega_norm * omega_norm));
      //  prime vertical radius
      //  Transform initial position from LLA to ECEF
      theta = std::cos(GNSS->lla1[0]);
      yaw_num = std::cos(GNSS->lla1[1]);
      yaw_den = std::sin(GNSS->lla1[1]);
      //  Initial ECEF position = origin of ENU frame
      a = (N + GNSS->lla1[2]) * theta;
      omega[0] = a * yaw_num;
      omega[1] = a * yaw_den;
      omega[2] = (N * 0.99330562000985867 + GNSS->lla1[2]) * omega_norm;
      //  Construct rotation transformation from ECEF to ENU
      R_position1[0] = -yaw_den;
      R_position1[3] = yaw_num;
      R_position1[6] = 0.0;
      R_position1[1] = -omega_norm * yaw_num;
      R_position1[4] = -omega_norm * yaw_den;
      R_position1[7] = theta;
      R_position1[2] = theta * yaw_num;
      R_position1[5] = theta * yaw_den;
      R_position1[8] = omega_norm;
      //  Homogeneous transformation:
      //
      //  p_enu =
      //  R_enu_ecef * p_ecef
      //  - R_enu_ecef * p_ecef0
      //
      //  therefore:
      //
      //  t = -R * p_ecef0
      //  Construct fixed homogeneous ECEF -> ENU transformation
      std::memset(&T_enu_ecef[0], 0, 16U * sizeof(double));
      for (int R_measurement_tmp{0}; R_measurement_tmp < 3;
           R_measurement_tmp++) {
        k = R_measurement_tmp << 2;
        T_enu_ecef[k] = R_position1[3 * R_measurement_tmp];
        T_enu_ecef[k + 1] = R_position1[3 * R_measurement_tmp + 1];
        T_enu_ecef[k + 2] = R_position1[3 * R_measurement_tmp + 2];
      }
      for (int R_measurement_tmp{0}; R_measurement_tmp < 9;
           R_measurement_tmp++) {
        R_position1[R_measurement_tmp] = -R_position1[R_measurement_tmp];
      }
      T_enu_ecef[15] = 1.0;
      //  Store ENU origin expressed in ECEF
      prev_timestamp = IMU->timestamp;
      //  EKF func is triggered by avaible IMU data
      //
      //  Inputs:
      //    x_sensor_body = sensor x-axis expressed in body coordinates
      //    y_sensor_body = sensor y-axis expressed in body coordinates
      //    z_sensor_body = sensor z-axis expressed in body coordinates
      //
      //  Output:
      //    R_body_sensor = rotation matrix Sensor -> Body
      a = 0.0;
      N = 3.3121686421112381E-170;
      yaw_den = 0.0;
      omega_norm = 3.3121686421112381E-170;
      theta = 0.0;
      yaw_num = 3.3121686421112381E-170;
      for (k = 0; k < 3; k++) {
        double absxk;
        double t;
        T_enu_ecef[k + 12] =
            (R_position1[k] * omega[0] + R_position1[k + 3] * omega[1]) +
            R_position1[k + 6] * omega[2];
        state->p_ecef0[k] = omega[k];
        absxk = std::abs(IMU->mounting.x_sensor_body[k]);
        if (absxk > N) {
          t = N / absxk;
          a = a * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          a += t * t;
        }
        absxk = std::abs(IMU->mounting.y_sensor_body[k]);
        if (absxk > omega_norm) {
          t = omega_norm / absxk;
          yaw_den = yaw_den * t * t + 1.0;
          omega_norm = absxk;
        } else {
          t = absxk / omega_norm;
          yaw_den += t * t;
        }
        absxk = std::abs(IMU->mounting.z_sensor_body[k]);
        if (absxk > yaw_num) {
          t = yaw_num / absxk;
          theta = theta * t * t + 1.0;
          yaw_num = absxk;
        } else {
          t = absxk / yaw_num;
          theta += t * t;
        }
      }
      a = N * std::sqrt(a);
      yaw_den = omega_norm * std::sqrt(yaw_den);
      theta = yaw_num * std::sqrt(theta);
      R_body_sensor[0] = IMU->mounting.x_sensor_body[0] / a;
      R_body_sensor[3] = IMU->mounting.y_sensor_body[0] / yaw_den;
      R_body_sensor[6] = IMU->mounting.z_sensor_body[0] / theta;
      R_body_sensor[1] = IMU->mounting.x_sensor_body[1] / a;
      R_body_sensor[4] = IMU->mounting.y_sensor_body[1] / yaw_den;
      R_body_sensor[7] = IMU->mounting.z_sensor_body[1] / theta;
      R_body_sensor[2] = IMU->mounting.x_sensor_body[2] / a;
      R_body_sensor[5] = IMU->mounting.y_sensor_body[2] / yaw_den;
      R_body_sensor[8] = IMU->mounting.z_sensor_body[2] / theta;
    } else {
      double F[100];
      double P_posterior[100];
      double b_F[100];
      double N;
      double a;
      double absxk;
      double d;
      double omega_norm;
      double q_prior_idx_0;
      double q_prior_idx_1;
      double q_prior_idx_2;
      double q_prior_idx_3;
      double t;
      double theta;
      double yaw_den;
      double yaw_num;
      int R_measurement_tmp;
      int b_R_measurement_tmp;
      int k;
      //  =========================================================
      //  PREDICTION BY IMU GYROSCOPE + BICYCLE MODEL
      //  ==========================================================
      if (IMU->valid) {
        double b_G_gyro[30];
        t = (IMU->timestamp - prev_timestamp) / 1000.0;
        // [sec]
        prev_timestamp = IMU->timestamp;
        // ms
        //  IMU gives angular velocity in SENSOR frame
        //  Transform the same angular velocity into BODY frame
        //
        //  omega_sensor = angular velocity measured in sensor frame
        //  omega_body   = same angular velocity expressed in body frame
        //  prediction_step expects gyro measurement in BODY frame
        //
        //  State:
        //
        //  state.p  = [E; N; U]             3x1
        //  state.q  = [qw; qx; qy; qz]     4x1
        //  state.bg = gyro bias             3x1
        //  state.P  = covariance            10x10
        //  state.Q  = process noise         10x10
        //
        //  Vehicle:
        //
        //  Vehicle.velocity       = vehicle speed [m/s]
        //  Vehicle.steering_angle = steering angle [rad]
        //  Vehicle.wheelbase      = wheelbase [m]
        //
        //  State ordering:
        //
        //  x =
        //  [ position    1:3
        //    quaternion  4:7
        //    gyro bias   8:10 ]
        //  =========================================================
        //  transfroming sensor frame measurements into body-frame
        //  =========================================================
        //  1. SAVE PREVIOUS STATE
        //  ==========================================================
        //  =========================================================
        //  2. REMOVE ESTIMATED GYRO BIAS
        //  ==========================================================
        d = IMU->gyro[0];
        yaw_num = IMU->gyro[1];
        yaw_den = IMU->gyro[2];
        for (R_measurement_tmp = 0; R_measurement_tmp < 3;
             R_measurement_tmp++) {
          omega[R_measurement_tmp] =
              ((R_body_sensor[R_measurement_tmp] * d +
                R_body_sensor[R_measurement_tmp + 3] * yaw_num) +
               R_body_sensor[R_measurement_tmp + 6] * yaw_den) -
              state->bg[R_measurement_tmp];
        }
        //  best estimate for angular veolcity
        //  =========================================================
        //  3. CALCULATE ROTATION DURING dt
        //  ==========================================================
        omega_norm = std::sqrt((omega[0] * omega[0] + omega[1] * omega[1]) +
                               omega[2] * omega[2]);
        if (omega_norm > 1.0E-8) {
          //  Total rotation angle during dt (during dt ang vel is supposed to
          //  be constant)
          theta = omega_norm * t;
          //  Rotation axis
          //  Incremental quaternion : rotation of angle theta around u axis
          yaw_num = std::sin(theta / 2.0);
          delta_q[0] = std::cos(theta / 2.0);
          delta_q[1] = omega[0] / omega_norm * yaw_num;
          delta_q[2] = omega[1] / omega_norm * yaw_num;
          delta_q[3] = omega[2] / omega_norm * yaw_num;
        } else {
          delta_q[0] = 1.0;
          delta_q[1] = 0.0;
          delta_q[2] = 0.0;
          delta_q[3] = 0.0;
        }
        //  =========================================================
        //  4. QUATERNION PREDICTION
        //
        //  q_prior = q_previous (*) delta_q
        //  ==========================================================
        q_prior_idx_0 = ((state->q[0] * delta_q[0] - state->q[1] * delta_q[1]) -
                         state->q[2] * delta_q[2]) -
                        state->q[3] * delta_q[3];
        q_prior_idx_1 = ((state->q[0] * delta_q[1] + delta_q[0] * state->q[1]) +
                         state->q[2] * delta_q[3]) -
                        delta_q[2] * state->q[3];
        q_prior_idx_2 = ((state->q[0] * delta_q[2] - state->q[1] * delta_q[3]) +
                         delta_q[0] * state->q[2]) +
                        delta_q[1] * state->q[3];
        q_prior_idx_3 = ((state->q[0] * delta_q[3] + state->q[1] * delta_q[2]) -
                         delta_q[1] * state->q[2]) +
                        delta_q[0] * state->q[3];
        //  =========================================================
        //  5. NORMALIZE QUATERNION : bcs of numerical instability and valid
        //  rotation must satistfies  ||q|| = 1
        //  ==========================================================
        omega_norm = std::sqrt(
            ((q_prior_idx_0 * q_prior_idx_0 + q_prior_idx_1 * q_prior_idx_1) +
             q_prior_idx_2 * q_prior_idx_2) +
            q_prior_idx_3 * q_prior_idx_3);
        q_prior_idx_0 /= omega_norm;
        q_prior_idx_1 /= omega_norm;
        q_prior_idx_2 /= omega_norm;
        q_prior_idx_3 /= omega_norm;
        //  =========================================================
        //  6. POSITION PREDICTION USING BICYCLE MODEL
        //  ==========================================================
        //  ---------------------------------------------------------
        //  6.1 Extract ENU yaw from previous quaternion
        //  ---------------------------------------------------------
        //
        //  psi = atan2(R21, R11)
        //
        //  where
        //
        //  R21 = 2(qw*qz + qx*qy)
        //  R11 = 1 - 2(qy^2 + qz^2)
        yaw_num = 2.0 * (state->q[0] * state->q[3] + state->q[1] * state->q[2]);
        yaw_den =
            1.0 - 2.0 * (state->q[2] * state->q[2] + state->q[3] * state->q[3]);
        //  ---------------------------------------------------------
        //  6.2 Bicycle-model yaw rate
        //  ---------------------------------------------------------
        //
        //  psi_dot = v/L * tan(delta)
        //  ---------------------------------------------------------
        //  6.3 Heading at middle of integration interval
        //  ---------------------------------------------------------
        //
        //  psi_mid =
        //
        //  psi + 1/2 * psi_dot * dt
        omega_norm = rt_atan2d_snf(yaw_num, yaw_den) +
                     0.5 *
                         (Vehicle->velocity / Vehicle->wheelbase *
                          std::tan(Vehicle->steering_angle)) *
                         t;
        //  ---------------------------------------------------------
        //  6.4 Predict ENU position
        //  ---------------------------------------------------------
        //
        //  E_k = E_(k-1) + v*cos(psi_mid)*dt
        //
        //  N_k = N_(k-1) + v*sin(psi_mid)*dt
        //
        //  U_k = U_(k-1)
        a = Vehicle->velocity * std::cos(omega_norm) * t;
        state->p[0] += a;
        omega_norm = std::sin(omega_norm);
        state->p[1] += Vehicle->velocity * omega_norm * t;
        //  =========================================================
        //  7. GYRO BIAS PREDICTION
        //  ==========================================================
        //
        //  Random-walk nominal model:
        //
        //  bg_k^- = bg_(k-1)^+
        //  =========================================================
        //  8. CREATE STATE TRANSITION JACOBIAN F
        //  ==========================================================
        for (R_measurement_tmp = 0; R_measurement_tmp < 100;
             R_measurement_tmp++) {
          F[R_measurement_tmp] = b_I[R_measurement_tmp];
        }
        //  ---------------------------------------------------------
        //  8.1 POSITION -> POSITION
        //  ---------------------------------------------------------
        //
        //  p_k contains p_(k-1), therefore:
        //
        //  dp_k / dp_(k-1) = I
        for (R_measurement_tmp = 0; R_measurement_tmp < 3;
             R_measurement_tmp++) {
          F[10 * R_measurement_tmp] = iv[3 * R_measurement_tmp];
          F[10 * R_measurement_tmp + 1] = iv[3 * R_measurement_tmp + 1];
          F[10 * R_measurement_tmp + 2] = iv[3 * R_measurement_tmp + 2];
        }
        //  ---------------------------------------------------------
        //  8.2 QUATERNION -> POSITION
        //  ---------------------------------------------------------
        //
        //  Position depends on:
        //
        //  psi = psi(q)
        //
        //  Therefore:
        //
        //  dp_k/dq =
        //
        //  dp_k/dpsi * dpsi/dq
        //  psi = atan2(A, B)
        //
        //  A = 2(qw*qz + qx*qy)
        //  B = 1 - 2(qy^2 + qz^2)
        theta = yaw_num * yaw_num + yaw_den * yaw_den;
        //  Derivatives of A with respect to quaternion
        //  Derivatives of B with respect to quaternion
        //  Derivative of atan2(A,B):
        //
        //  dpsi/dq =
        //
        //  (B*dA/dq - A*dB/dq)
        //  ---------------------
        //        A^2 + B^2
        delta_q[0] = (yaw_den * (2.0 * state->q[3]) - yaw_num * 0.0) / theta;
        delta_q[1] = (yaw_den * (2.0 * state->q[2]) - yaw_num * 0.0) / theta;
        delta_q[2] =
            (yaw_den * (2.0 * state->q[1]) - yaw_num * (-4.0 * state->q[2])) /
            theta;
        delta_q[3] =
            (yaw_den * (2.0 * state->q[0]) - yaw_num * (-4.0 * state->q[3])) /
            theta;
        //  Position derivative with respect to yaw
        //
        //  E = E_prev + v*cos(psi_mid)*dt
        //
        //  N = N_prev + v*sin(psi_mid)*dt
        omega_norm = -Vehicle->velocity * omega_norm * t;
        //  Chain rule:
        //
        //  dp/dq = dp/dpsi * dpsi/dq
        F[30] = omega_norm * delta_q[0];
        F[31] = a * delta_q[0];
        F[32] = 0.0;
        F[40] = omega_norm * delta_q[1];
        F[41] = a * delta_q[1];
        F[42] = 0.0;
        F[50] = omega_norm * delta_q[2];
        F[51] = a * delta_q[2];
        F[52] = 0.0;
        F[60] = omega_norm * delta_q[3];
        F[61] = a * delta_q[3];
        F[62] = 0.0;
        //  ---------------------------------------------------------
        //  8.3 QUATERNION -> QUATERNION
        //  ---------------------------------------------------------
        omega_norm = 0.0 * t;
        theta = 0.5 * -omega[0] * t;
        yaw_num = 0.5 * -omega[1] * t;
        yaw_den = 0.5 * -omega[2] * t;
        a = 0.5 * omega[0] * t;
        N = 0.5 * omega[2] * t;
        absxk = 0.5 * omega[1] * t;
        F[33] = omega_norm + 1.0;
        F[34] = a;
        F[35] = absxk;
        F[36] = N;
        F[43] = theta;
        F[44] = omega_norm + 1.0;
        F[45] = yaw_den;
        F[46] = absxk;
        F[53] = yaw_num;
        F[54] = N;
        F[55] = omega_norm + 1.0;
        F[56] = theta;
        F[63] = yaw_den;
        F[64] = yaw_num;
        F[65] = a;
        F[66] = omega_norm + 1.0;
        //  ---------------------------------------------------------
        //  8.4 GYRO BIAS -> QUATERNION
        //  ---------------------------------------------------------
        omega_norm = -0.5 * -state->q[1] * t;
        F[73] = omega_norm;
        theta = -0.5 * -state->q[2] * t;
        F[83] = theta;
        yaw_den = -0.5 * -state->q[3] * t;
        F[93] = yaw_den;
        yaw_num = -0.5 * state->q[0] * t;
        F[74] = yaw_num;
        F[84] = yaw_den;
        F[94] = -0.5 * state->q[2] * t;
        F[75] = -0.5 * state->q[3] * t;
        F[85] = yaw_num;
        F[95] = omega_norm;
        F[76] = theta;
        F[86] = -0.5 * state->q[1] * t;
        F[96] = yaw_num;
        //  =========================================================
        //  9. GYRO MEASUREMENT NOISE CONTRIBUTION
        //  ==========================================================
        //
        //  Calculates the contribution of IMU gyroscope measurement noise
        //  to the EKF prediction covariance.
        //
        //  q           : Body -> ENU quaternion [qw; qx; qy; qz]
        //  Sigma_gyro  : 3x3 gyro measurement covariance [(rad/s)^2]
        //  dt          : prediction timestep [s]
        //
        //  Q_gyro      : 10x10 state process-noise contribution
        //  Quaternion sensitivity to angular velocity
        //  Noise-input Jacobian
        //
        //  State ordering:
        //
        //  x = [p(3); q(4); bg(3)]
        //
        //  Gyro noise directly affects only the quaternion.
        std::memset(&G_gyro[0], 0, 30U * sizeof(double));
        omega_norm = 0.5 * -state->q[1] * t;
        G_gyro[3] = omega_norm;
        theta = 0.5 * -state->q[2] * t;
        G_gyro[13] = theta;
        yaw_num = 0.5 * -state->q[3] * t;
        G_gyro[23] = yaw_num;
        yaw_den = 0.5 * state->q[0] * t;
        G_gyro[4] = yaw_den;
        G_gyro[14] = yaw_num;
        G_gyro[24] = 0.5 * state->q[2] * t;
        G_gyro[5] = 0.5 * state->q[3] * t;
        G_gyro[15] = yaw_den;
        G_gyro[25] = omega_norm;
        G_gyro[6] = theta;
        G_gyro[16] = 0.5 * state->q[1] * t;
        G_gyro[26] = yaw_den;
        //  Gyroscope contribution to prediction covariance
        //  =========================================================
        //  10. COVARIANCE PREDICTION
        //  ==========================================================
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 10; k++) {
              d += F[R_measurement_tmp + 10 * k] *
                   state->P[k + 10 * b_R_measurement_tmp];
            }
            b_F[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
          d = G_gyro[R_measurement_tmp];
          yaw_num = G_gyro[R_measurement_tmp + 10];
          yaw_den = G_gyro[R_measurement_tmp + 20];
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 3;
               b_R_measurement_tmp++) {
            b_G_gyro[R_measurement_tmp + 10 * b_R_measurement_tmp] =
                (d * IMU->gyro_covariance[3 * b_R_measurement_tmp] +
                 yaw_num * IMU->gyro_covariance[3 * b_R_measurement_tmp + 1]) +
                yaw_den * IMU->gyro_covariance[3 * b_R_measurement_tmp + 2];
          }
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 10; k++) {
              d += b_F[R_measurement_tmp + 10 * k] *
                   F[b_R_measurement_tmp + 10 * k];
            }
            P_posterior[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          d = b_G_gyro[R_measurement_tmp];
          yaw_num = b_G_gyro[R_measurement_tmp + 10];
          yaw_den = b_G_gyro[R_measurement_tmp + 20];
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            F[R_measurement_tmp + 10 * b_R_measurement_tmp] =
                (d * G_gyro[b_R_measurement_tmp] +
                 yaw_num * G_gyro[b_R_measurement_tmp + 10]) +
                yaw_den * G_gyro[b_R_measurement_tmp + 20];
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 100;
             R_measurement_tmp++) {
          state->P[R_measurement_tmp] =
              (P_posterior[R_measurement_tmp] + F[R_measurement_tmp]) +
              state->Q[R_measurement_tmp];
        }
        //  =========================================================
        //  11. SAVE A PRIORI STATE
        //  ==========================================================
        state->q[0] = q_prior_idx_0;
        state->q[1] = q_prior_idx_1;
        state->q[2] = q_prior_idx_2;
        state->q[3] = q_prior_idx_3;
      }
      //  =========================================================
      //  GNSS CORRECTION
      //  ==========================================================
      if (GNSS->valid) {
        double K[60];
        double b_H[60];
        double y_tmp[60];
        double R_measurement[36];
        double c_H[36];
        double delta_x[10];
        double b_delta_q[6];
        double dv[4];
        double q_idx_0;
        double q_idx_1;
        double q_idx_2;
        //  GNSS antenna 1: LLA -> ECEF -> fixed ENU
        //
        //  Converts geodetic LLA coordinates to ECEF.
        //
        //  Input:
        //    lla = [latitude;
        //           longitude;
        //           altitude]
        //
        //    latitude  [rad]
        //    longitude [rad]
        //    altitude  [m]
        //
        //  Output:
        //    p_ecef = [X;
        //              Y;
        //              Z]          [m]
        //  =========================================================
        //  1. EXTRACT LLA
        //  ==========================================================
        //  =========================================================
        //  2. WGS-84 CONSTANTS
        //  ==========================================================
        //  =========================================================
        //  3. PRIME VERTICAL RADIUS
        //  ==========================================================
        theta = std::sin(GNSS->lla1[0]);
        N = 6.378137E+6 /
            std::sqrt(1.0 - 0.0066943799901413165 * theta * theta);
        //  =========================================================
        //  4. LLA -> ECEF
        //  ==========================================================
        //  =========================================================
        //  5. OUTPUT VECTOR
        //  ==========================================================
        omega_norm = (N + GNSS->lla1[2]) * std::cos(GNSS->lla1[0]);
        yaw_den = omega_norm * std::cos(GNSS->lla1[1]);
        yaw_num = omega_norm * std::sin(GNSS->lla1[1]);
        omega_norm = (N * 0.99330562000985867 + GNSS->lla1[2]) * theta;
        //  GNSS antenna 2: LLA -> ECEF -> fixed ENU
        //  Baseline measured directly in fixed ENU frame
        //  Position used for absolute-position correction
        //  EKF correction
        //
        //  state:
        //    p  : 3x1 ENU vehicle position
        //    q  : 4x1 Body -> ENU quaternion [qw qx qy qz]'
        //    bg : 3x1 gyro bias
        //    P  : 10x10 a priori covariance
        //
        //  GNSS:
        //    position_enu  : 3x1 vehicle/body-reference position in ENU
        //    position1_enu : 3x1 GNSS antenna 1 position in ENU
        //    position2_enu : 3x1 GNSS antenna 2 position in ENU
        //    baseline_body : known P1 -> P2 baseline in body frame
        //
        //  Example for:
        //
        //        +x_B forward
        //             ^
        //             |
        //    P2 o-----+-----o P1
        //
        //  P1 = right antenna
        //  P2 = left antenna
        //
        //  baseline_body = [0; B; 0]
        //  =========================================================
        //  1. SAVE A PRIORI STATE
        //  =========================================================
        //  2. NORMALIZE A PRIORI QUATERNION
        //  =========================================================
        a = 0.0;
        N = 3.3121686421112381E-170;
        for (k = 0; k < 4; k++) {
          delta_q[k] =
              ((T_enu_ecef[k] * yaw_den + T_enu_ecef[k + 4] * yaw_num) +
               T_enu_ecef[k + 8] * omega_norm) +
              T_enu_ecef[k + 12];
          absxk = std::abs(state->q[k]);
          if (absxk > N) {
            t = N / absxk;
            a = a * t * t + 1.0;
            N = absxk;
          } else {
            t = absxk / N;
            a += t * t;
          }
        }
        a = N * std::sqrt(a);
        //  =========================================================
        //  3. POSITION INNOVATION
        //  =========================================================
        //  4. DUAL-GNSS BASELINE INNOVATION
        //
        //  measured:
        //
        //      b_meas = p2 - p1
        //
        //  predicted:
        //
        //      b_pred = R_ENU_BODY(q_prior) * b_body
        //
        //  innovation:
        //
        //      y_b = b_meas - b_pred
        //
        //  Also calculate:
        //
        //      H_bq = d(b_pred)/dq
        //  =========================================================
        //
        //  Dual-GNSS baseline measurement model.
        //
        //  Inputs:
        //
        //  p1_ENU : 3x1 position of antenna 1 in ENU
        //  p2_ENU : 3x1 position of antenna 2 in ENU
        //
        //  bBody :
        //
        //  known antenna baseline in body coordinates
        //
        //      bBody = p2_body - p1_body
        //
        //  q :
        //
        //  Body -> ENU quaternion
        //
        //      q = [qw qx qy qz]'
        //  =========================================================
        //  1. MEASURED BASELINE
        //  =========================================================
        //  2. NORMALIZE QUATERNION
        //  =========================================================
        N = 3.3121686421112381E-170;
        d = state->q[0] / a;
        q_prior_idx_0 = d;
        absxk = std::abs(d);
        if (absxk > 3.3121686421112381E-170) {
          yaw_den = 1.0;
          N = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          yaw_den = t * t;
        }
        d = state->q[1] / a;
        q_prior_idx_1 = d;
        absxk = std::abs(d);
        if (absxk > N) {
          t = N / absxk;
          yaw_den = yaw_den * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          yaw_den += t * t;
        }
        d = state->q[2] / a;
        q_prior_idx_2 = d;
        absxk = std::abs(d);
        if (absxk > N) {
          t = N / absxk;
          yaw_den = yaw_den * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          yaw_den += t * t;
        }
        d = state->q[3] / a;
        absxk = std::abs(d);
        if (absxk > N) {
          t = N / absxk;
          yaw_den = yaw_den * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          yaw_den += t * t;
        }
        yaw_den = N * std::sqrt(yaw_den);
        q_idx_0 = q_prior_idx_0 / yaw_den;
        q_idx_1 = q_prior_idx_1 / yaw_den;
        q_idx_2 = q_prior_idx_2 / yaw_den;
        q_prior_idx_3 = d / yaw_den;
        //  =========================================================
        //  3. BODY -> ENU ROTATION MATRIX
        //  =========================================================
        //  4. PREDICTED BASELINE
        //
        //  h_b(x) =
        //
        //  R_ENU_BODY(q) * b_body
        //  =========================================================
        //  5. BASELINE INNOVATION
        //
        //  y_b =
        //
        //  measured - predicted
        //  =========================================================
        //  6. JACOBIAN OF PREDICTED BASELINE W.R.T QUATERNION
        //
        //  H_bq =
        //
        //  d [R(q)b] / dq
        //
        //  size = 3x4
        //  =========================================================
        //  5. COMPLETE INNOVATION VECTOR
        //  =========================================================
        //  6. MEASUREMENT JACOBIAN
        //
        //  State ordering:
        //
        //  x = [ p(3)
        //        q(4)
        //        bg(3) ]
        //
        //  total = 10 states
        //  =========================================================
        std::memset(&H[0], 0, 60U * sizeof(double));
        //  ---------------------------------------------------------
        //  Position measurement
        //
        //  h_p(x) = p
        //
        //  therefore:
        //
        //  dh_p / dp = I
        //  ---------------------------------------------------------
        for (R_measurement_tmp = 0; R_measurement_tmp < 3;
             R_measurement_tmp++) {
          H[6 * R_measurement_tmp] = iv[3 * R_measurement_tmp];
          H[6 * R_measurement_tmp + 1] = iv[3 * R_measurement_tmp + 1];
          H[6 * R_measurement_tmp + 2] = iv[3 * R_measurement_tmp + 2];
        }
        //  ---------------------------------------------------------
        //  Baseline measurement
        //
        //  h_b(x) =
        //
        //  R_ENU_BODY(q) * b_body
        //
        //  Only depends on quaternion in this model
        //  ---------------------------------------------------------
        omega_norm = q_idx_2 * GNSS->baseline_body[2];
        H[21] = 2.0 * (-q_prior_idx_3 * GNSS->baseline_body[1] + omega_norm);
        theta = GNSS->baseline_body[2] * q_prior_idx_3;
        yaw_num = GNSS->baseline_body[1] * q_idx_2;
        H[27] = 2.0 * (yaw_num + theta);
        yaw_den = q_idx_0 * GNSS->baseline_body[2];
        a = q_idx_1 * GNSS->baseline_body[1];
        H[33] = 2.0 * ((-2.0 * q_idx_2 * GNSS->baseline_body[0] + a) + yaw_den);
        N = q_idx_1 * GNSS->baseline_body[2];
        absxk = q_idx_0 * GNSS->baseline_body[1];
        H[39] =
            2.0 * ((-2.0 * q_prior_idx_3 * GNSS->baseline_body[0] - absxk) + N);
        t = GNSS->baseline_body[0] * q_prior_idx_3;
        H[22] = 2.0 * (t - N);
        H[28] = 2.0 * ((GNSS->baseline_body[0] * q_idx_2 -
                        2.0 * q_idx_1 * GNSS->baseline_body[1]) -
                       yaw_den);
        yaw_den = GNSS->baseline_body[0] * q_idx_1;
        H[34] = 2.0 * (yaw_den + theta);
        H[40] = 2.0 * ((q_idx_0 * GNSS->baseline_body[0] -
                        2.0 * q_prior_idx_3 * GNSS->baseline_body[1]) +
                       omega_norm);
        H[23] = 2.0 * (-q_idx_2 * GNSS->baseline_body[0] + a);
        H[29] = 2.0 * ((t + absxk) - 2.0 * q_idx_1 * GNSS->baseline_body[2]);
        H[35] = 2.0 * ((-q_idx_0 * GNSS->baseline_body[0] +
                        GNSS->baseline_body[1] * q_prior_idx_3) -
                       2.0 * q_idx_2 * GNSS->baseline_body[2]);
        H[41] = 2.0 * (yaw_den + yaw_num);
        //  =========================================================
        //  7. MEASUREMENT COVARIANCE
        //  =========================================================
        a = GNSS->hAcc1 * GNSS->hAcc1;
        std::memset(&R_position1[0], 0, 9U * sizeof(double));
        R_position1[0] = a;
        R_position1[4] = a;
        R_position1[8] = GNSS->vAcc1 * GNSS->vAcc1;
        a = GNSS->hAcc2 * GNSS->hAcc2;
        omega[0] = a;
        omega[1] = a;
        omega[2] = GNSS->vAcc2 * GNSS->vAcc2;
        std::memset(&R_position2[0], 0, 9U * sizeof(double));
        //  This covariance corresponds to measurement vector:
        //
        //  z =
        //
        //  [ p1
        //    p2 - p1 ]
        //
        //  assuming receiver errors are independent.
        for (int j{0}; j < 3; j++) {
          R_position2[j + 3 * j] = omega[j];
          yaw_num = R_position1[3 * j];
          R_measurement[6 * j] = yaw_num;
          k = 6 * (j + 3);
          R_measurement[k] = -yaw_num;
          R_measurement[6 * j + 3] = -yaw_num;
          b_R_measurement_tmp = 3 * j + 1;
          yaw_den = R_position1[b_R_measurement_tmp];
          R_measurement[6 * j + 1] = yaw_den;
          R_measurement[k + 1] = -yaw_den;
          R_measurement[6 * j + 4] = -yaw_den;
          R_measurement_tmp = 3 * j + 2;
          theta = R_position1[R_measurement_tmp];
          R_measurement[6 * j + 2] = theta;
          R_measurement[k + 2] = -theta;
          R_measurement[6 * j + 5] = -theta;
          R_measurement[k + 3] = yaw_num + R_position2[3 * j];
          R_measurement[k + 4] = yaw_den + R_position2[b_R_measurement_tmp];
          R_measurement[k + 5] = theta + R_position2[R_measurement_tmp];
        }
        //  =========================================================
        //  8. INNOVATION COVARIANCE
        //  =========================================================
        for (R_measurement_tmp = 0; R_measurement_tmp < 6;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            y_tmp[b_R_measurement_tmp + 10 * R_measurement_tmp] =
                H[R_measurement_tmp + 6 * b_R_measurement_tmp];
          }
        }
        //  =========================================================
        //  9. KALMAN GAIN
        //  =========================================================
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 6;
               b_R_measurement_tmp++) {
            yaw_num = 0.0;
            for (k = 0; k < 10; k++) {
              yaw_num += state->P[R_measurement_tmp + 10 * k] *
                         y_tmp[k + 10 * b_R_measurement_tmp];
            }
            K[R_measurement_tmp + 10 * b_R_measurement_tmp] = yaw_num;
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 6;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            yaw_num = 0.0;
            for (k = 0; k < 10; k++) {
              yaw_num += H[R_measurement_tmp + 6 * k] *
                         state->P[k + 10 * b_R_measurement_tmp];
            }
            b_H[R_measurement_tmp + 6 * b_R_measurement_tmp] = yaw_num;
          }
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 6;
               b_R_measurement_tmp++) {
            yaw_num = 0.0;
            for (k = 0; k < 10; k++) {
              yaw_num += b_H[R_measurement_tmp + 6 * k] *
                         y_tmp[k + 10 * b_R_measurement_tmp];
            }
            k = R_measurement_tmp + 6 * b_R_measurement_tmp;
            c_H[k] = yaw_num + R_measurement[k];
          }
        }
        coder::internal::mrdiv(K, c_H);
        //  =========================================================
        //  10. STATE CORRECTION
        //  =========================================================
        //
        //  Converts geodetic LLA coordinates to ECEF.
        //
        //  Input:
        //    lla = [latitude;
        //           longitude;
        //           altitude]
        //
        //    latitude  [rad]
        //    longitude [rad]
        //    altitude  [m]
        //
        //  Output:
        //    p_ecef = [X;
        //              Y;
        //              Z]          [m]
        //  =========================================================
        //  1. EXTRACT LLA
        //  ==========================================================
        //  =========================================================
        //  2. WGS-84 CONSTANTS
        //  ==========================================================
        //  =========================================================
        //  3. PRIME VERTICAL RADIUS
        //  ==========================================================
        theta = std::sin(GNSS->lla2[0]);
        N = 6.378137E+6 /
            std::sqrt(1.0 - 0.0066943799901413165 * theta * theta);
        //  =========================================================
        //  4. LLA -> ECEF
        //  ==========================================================
        //  =========================================================
        //  5. OUTPUT VECTOR
        //  ==========================================================
        omega_norm = (N + GNSS->lla2[2]) * std::cos(GNSS->lla2[0]);
        yaw_den = omega_norm * std::cos(GNSS->lla2[1]);
        yaw_num = omega_norm * std::sin(GNSS->lla2[1]);
        omega_norm = (N * 0.99330562000985867 + GNSS->lla2[2]) * theta;
        for (R_measurement_tmp = 0; R_measurement_tmp < 4;
             R_measurement_tmp++) {
          dv[R_measurement_tmp] =
              ((T_enu_ecef[R_measurement_tmp] * yaw_den +
                T_enu_ecef[R_measurement_tmp + 4] * yaw_num) +
               T_enu_ecef[R_measurement_tmp + 8] * omega_norm) +
              T_enu_ecef[R_measurement_tmp + 12];
        }
        omega_norm = q_prior_idx_3 * q_prior_idx_3;
        theta = q_idx_2 * q_idx_2;
        R_position1[0] = 1.0 - 2.0 * (theta + omega_norm);
        yaw_num = q_idx_1 * q_idx_2;
        yaw_den = q_idx_0 * q_prior_idx_3;
        R_position1[3] = 2.0 * (yaw_num - yaw_den);
        a = q_idx_1 * q_prior_idx_3;
        N = q_idx_0 * q_idx_2;
        R_position1[6] = 2.0 * (a + N);
        R_position1[1] = 2.0 * (yaw_num + yaw_den);
        yaw_num = q_idx_1 * q_idx_1;
        R_position1[4] = 1.0 - 2.0 * (yaw_num + omega_norm);
        omega_norm = q_idx_2 * q_prior_idx_3;
        yaw_den = q_idx_0 * q_idx_1;
        R_position1[7] = 2.0 * (omega_norm - yaw_den);
        R_position1[2] = 2.0 * (a - N);
        R_position1[5] = 2.0 * (omega_norm + yaw_den);
        R_position1[8] = 1.0 - 2.0 * (yaw_num + theta);
        yaw_num = GNSS->baseline_body[0];
        yaw_den = GNSS->baseline_body[1];
        theta = GNSS->baseline_body[2];
        for (R_measurement_tmp = 0; R_measurement_tmp < 3;
             R_measurement_tmp++) {
          omega_norm = delta_q[R_measurement_tmp];
          b_delta_q[R_measurement_tmp] =
              omega_norm - state->p[R_measurement_tmp];
          b_delta_q[R_measurement_tmp + 3] =
              (dv[R_measurement_tmp] - omega_norm) -
              ((R_position1[R_measurement_tmp] * yaw_num +
                R_position1[R_measurement_tmp + 3] * yaw_den) +
               R_position1[R_measurement_tmp + 6] * theta);
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          yaw_num = 0.0;
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 6;
               b_R_measurement_tmp++) {
            yaw_num += K[R_measurement_tmp + 10 * b_R_measurement_tmp] *
                       b_delta_q[b_R_measurement_tmp];
          }
          delta_x[R_measurement_tmp] = yaw_num;
        }
        state->p[0] += delta_x[0];
        state->p[1] += delta_x[1];
        state->p[2] += delta_x[2];
        q_prior_idx_0 += delta_x[3];
        q_prior_idx_1 += delta_x[4];
        q_prior_idx_2 += delta_x[5];
        q_prior_idx_3 = d + delta_x[6];
        state->bg[0] += delta_x[7];
        state->bg[1] += delta_x[8];
        state->bg[2] += delta_x[9];
        //  =========================================================
        //  11. NORMALIZE QUATERNION
        //  =========================================================
        N = 3.3121686421112381E-170;
        absxk = std::abs(q_prior_idx_0);
        if (absxk > 3.3121686421112381E-170) {
          a = 1.0;
          N = absxk;
        } else {
          t = absxk / 3.3121686421112381E-170;
          a = t * t;
        }
        absxk = std::abs(q_prior_idx_1);
        if (absxk > N) {
          t = N / absxk;
          a = a * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          a += t * t;
        }
        absxk = std::abs(q_prior_idx_2);
        if (absxk > N) {
          t = N / absxk;
          a = a * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          a += t * t;
        }
        absxk = std::abs(q_prior_idx_3);
        if (absxk > N) {
          t = N / absxk;
          a = a * t * t + 1.0;
          N = absxk;
        } else {
          t = absxk / N;
          a += t * t;
        }
        a = N * std::sqrt(a);
        state->q[0] = q_prior_idx_0 / a;
        state->q[1] = q_prior_idx_1 / a;
        state->q[2] = q_prior_idx_2 / a;
        state->q[3] = q_prior_idx_3 / a;
        //  =========================================================
        //  12. JOSEPH-FORM COVARIANCE UPDATE
        //  =========================================================
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 6; k++) {
              d += K[R_measurement_tmp + 10 * k] *
                   H[k + 6 * b_R_measurement_tmp];
            }
            k = R_measurement_tmp + 10 * b_R_measurement_tmp;
            F[k] = static_cast<double>(b_I[k]) - d;
          }
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 10; k++) {
              d += F[R_measurement_tmp + 10 * k] *
                   state->P[k + 10 * b_R_measurement_tmp];
            }
            b_F[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 6;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 6; k++) {
              d += K[R_measurement_tmp + 10 * k] *
                   R_measurement[k + 6 * b_R_measurement_tmp];
            }
            y_tmp[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 10; k++) {
              d += b_F[R_measurement_tmp + 10 * k] *
                   F[b_R_measurement_tmp + 10 * k];
            }
            P_posterior[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            d = 0.0;
            for (k = 0; k < 6; k++) {
              d += y_tmp[R_measurement_tmp + 10 * k] *
                   K[b_R_measurement_tmp + 10 * k];
            }
            F[R_measurement_tmp + 10 * b_R_measurement_tmp] = d;
          }
        }
        for (R_measurement_tmp = 0; R_measurement_tmp < 100;
             R_measurement_tmp++) {
          P_posterior[R_measurement_tmp] += F[R_measurement_tmp];
        }
        //  Optional numerical symmetry enforcement
        for (R_measurement_tmp = 0; R_measurement_tmp < 10;
             R_measurement_tmp++) {
          for (b_R_measurement_tmp = 0; b_R_measurement_tmp < 10;
               b_R_measurement_tmp++) {
            k = b_R_measurement_tmp + 10 * R_measurement_tmp;
            state->P[k] =
                0.5 *
                (P_posterior[k] +
                 P_posterior[R_measurement_tmp + 10 * b_R_measurement_tmp]);
          }
        }
        //  =========================================================
        //  13. STORE POSTERIOR STATE
        //  =========================================================
      }
    }
  }
}

void EKF_func_init()
{
  prev_timestamp = -1.0;
  //  homogenius trasnfromation matrix
  std::memset(&T_enu_ecef[0], 0, 16U * sizeof(double));
  //  rotation transfroamtion matrix
  std::memset(&R_body_sensor[0], 0, 9U * sizeof(double));
}

// End of code generation (EKF_func.cpp)
