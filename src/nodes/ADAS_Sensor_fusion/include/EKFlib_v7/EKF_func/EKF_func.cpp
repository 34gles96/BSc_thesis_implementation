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
#include "norm.h"
#include "rt_nonfinite.h"
#include "rt_defines.h"
#include <cmath>
#include <cstring>

// Variable Definitions
static double prev_timestamp;

static double T_enu_ecef[16];

static double R_body_sensor[9];

static double prev_velocity;

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

void EKF_func(struct0_T *IMU, struct2_T *GNSS, const struct3_T *Vehicle,
              struct4_T *state)
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
  double H_q[9];
  double R_position1[9];
  double delta_q[4];
  double q_prior[4];
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
  //  =========================================================
  //  0. CONVERT EXTERNAL ROS GNSS INPUT
  //
  //  ROS NavSatFix:
  //  latitude  -> degrees
  //  longitude -> degrees
  //
  //  Inside the EKF all LLA angles are represented in radians.
  //  ==========================================================
  //
  //  Input:
  //    lla_deg = [latitude_deg;
  //               longitude_deg;
  //               altitude_m]
  //
  //  Output:
  //    lla_rad = [latitude_rad;
  //               longitude_rad;
  //               altitude_m]
  GNSS->lla1[0] = GNSS->lla1[0] * 3.1415926535897931 / 180.0;
  GNSS->lla1[1] = GNSS->lla1[1] * 3.1415926535897931 / 180.0;
  //  altitude remains in meters
  //
  //  Input:
  //    lla_deg = [latitude_deg;
  //               longitude_deg;
  //               altitude_m]
  //
  //  Output:
  //    lla_rad = [latitude_rad;
  //               longitude_rad;
  //               altitude_m]
  GNSS->lla2[0] = GNSS->lla2[0] * 3.1415926535897931 / 180.0;
  GNSS->lla2[1] = GNSS->lla2[1] * 3.1415926535897931 / 180.0;
  //  altitude remains in meters
  //
  //  hom trasnfromation matrix
  //  Debug
  // {
  // fprintf( ...
  //     "EKF call: IMU.valid=%d  GNSS.valid=%d  IMU.t=%.3f  prev_t=%.3f\n", ...
  //     IMU.valid, ...
  //     GNSS.valid, ...
  //     IMU.timestamp, ...
  //     prev_timestamp);
  // }
  //  initialization
  //  =========================================================
  //   WAITING FOR FIRST VALID GNSS
  //  ==========================================================
  //  in the first gnss data transmission: init of ENU origin
  //  =========================================================
  //  INITIALIZE FIXED ENU FRAME, ORIENTATION AND COVARIANCE
  //  =========================================================
  if (prev_timestamp < 0.0) {
    // disp(">>> INITIALIZATION BLOCK");
    if (GNSS->valid) {
      double dv[4];
      double absxk;
      double omega_norm;
      double psi_mid;
      double sinPhi;
      double t;
      double theta;
      double unnamed_idx_0_tmp;
      double yaw_den;
      int R_position1_tmp;
      //  Constant values during transformation from LLA to ECEF
      //  Earth flattening
      theta = std::sin(GNSS->lla1[0]);
      t = 6.378137E+6 /
          std::sqrt(1.0 - 0.0066943799901413165 * (theta * theta));
      //  prime vertical radius
      //  Transform initial position from LLA to ECEF
      psi_mid = std::cos(GNSS->lla1[0]);
      sinPhi = std::cos(GNSS->lla1[1]);
      omega_norm = std::sin(GNSS->lla1[1]);
      //  Initial ECEF position = origin of ENU frame
      absxk = (t + GNSS->lla1[2]) * psi_mid;
      omega[0] = absxk * sinPhi;
      omega[1] = absxk * omega_norm;
      omega[2] = (t * 0.99330562000985867 + GNSS->lla1[2]) * theta;
      //  Construct rotation transformation from ECEF to ENU
      R_position1[0] = -omega_norm;
      R_position1[3] = sinPhi;
      R_position1[6] = 0.0;
      R_position1[1] = -theta * sinPhi;
      R_position1[4] = -theta * omega_norm;
      R_position1[7] = psi_mid;
      R_position1[2] = psi_mid * sinPhi;
      R_position1[5] = psi_mid * omega_norm;
      R_position1[8] = theta;
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
      for (int i{0}; i < 3; i++) {
        R_position1_tmp = i << 2;
        T_enu_ecef[R_position1_tmp] = R_position1[3 * i];
        T_enu_ecef[R_position1_tmp + 1] = R_position1[3 * i + 1];
        T_enu_ecef[R_position1_tmp + 2] = R_position1[3 * i + 2];
      }
      for (int i{0}; i < 9; i++) {
        R_position1[i] = -R_position1[i];
      }
      T_enu_ecef[15] = 1.0;
      for (R_position1_tmp = 0; R_position1_tmp < 3; R_position1_tmp++) {
        T_enu_ecef[R_position1_tmp + 12] =
            (R_position1[R_position1_tmp] * omega[0] +
             R_position1[R_position1_tmp + 3] * omega[1]) +
            R_position1[R_position1_tmp + 6] * omega[2];
        state->p_ecef0[R_position1_tmp] = omega[R_position1_tmp];
      }
      //  GNSS 1: LLA -> ECEF -> ENU
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
      t = 6.378137E+6 / std::sqrt(1.0 - 0.0066943799901413165 * theta * theta);
      //  =========================================================
      //  4. LLA -> ECEF
      //  ==========================================================
      //  =========================================================
      //  5. OUTPUT VECTOR
      //  ==========================================================
      psi_mid *= t + GNSS->lla1[2];
      absxk = psi_mid * sinPhi;
      omega_norm *= psi_mid;
      sinPhi = (t * 0.99330562000985867 + GNSS->lla1[2]) * theta;
      for (int i{0}; i < 4; i++) {
        delta_q[i] = ((T_enu_ecef[i] * absxk + T_enu_ecef[i + 4] * omega_norm) +
                      T_enu_ecef[i + 8] * sinPhi) +
                     T_enu_ecef[i + 12];
      }
      //  GNSS 2: LLA -> ECEF -> ENU
      //  Baseline: right antenna -> left antenna
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
      sinPhi = std::sin(GNSS->lla2[0]);
      t = 6.378137E+6 /
          std::sqrt(1.0 - 0.0066943799901413165 * sinPhi * sinPhi);
      //  =========================================================
      //  4. LLA -> ECEF
      //  ==========================================================
      //  =========================================================
      //  5. OUTPUT VECTOR
      //  ==========================================================
      psi_mid = (t + GNSS->lla2[2]) * std::cos(GNSS->lla2[0]);
      absxk = psi_mid * std::cos(GNSS->lla2[1]);
      omega_norm = psi_mid * std::sin(GNSS->lla2[1]);
      sinPhi *= t * 0.99330562000985867 + GNSS->lla2[2];
      for (int i{0}; i < 4; i++) {
        dv[i] = ((T_enu_ecef[i] * absxk + T_enu_ecef[i + 4] * omega_norm) +
                 T_enu_ecef[i + 8] * sinPhi) +
                T_enu_ecef[i + 12];
      }
      //  Initial position
      omega[0] = dv[0] - delta_q[0];
      state->p[0] = delta_q[0];
      omega[1] = dv[1] - delta_q[1];
      state->p[1] = delta_q[1];
      state->p[2] = delta_q[2];
      //  Initial orientation
      omega_norm = rt_atan2d_snf(-omega[0], omega[1]);
      state->q[0] = std::cos(omega_norm / 2.0);
      state->q[1] = 0.0;
      state->q[2] = 0.0;
      state->q[3] = std::sin(omega_norm / 2.0);
      //  Initial covariance from GNSS
      //  =========================================================
      //  POSITION COVARIANCE
      //  =========================================================
      std::memset(&R_position1[0], 0, 9U * sizeof(double));
      theta = GNSS->hAcc1 * GNSS->hAcc1;
      R_position1[0] = theta;
      R_position1[4] = theta;
      R_position1[8] = GNSS->vAcc1 * GNSS->vAcc1;
      //  =========================================================
      //  BASELINE COVARIANCE
      //  =========================================================
      //  BASELINE -> YAW COVARIANCE
      //  =========================================================
      omega_norm = omega[0] * omega[0] + omega[1] * omega[1];
      sinPhi = -omega[1] / omega_norm;
      psi_mid = omega[0] / omega_norm;
      omega_norm = theta + GNSS->hAcc2 * GNSS->hAcc2;
      omega_norm = (sinPhi * omega_norm + psi_mid * 0.0) * sinPhi +
                   (sinPhi * 0.0 + psi_mid * omega_norm) * psi_mid;
      //  =========================================================
      //  YAW -> QUATERNION COVARIANCE
      //  =========================================================
      delta_q[0] = -0.5 * state->q[3];
      delta_q[1] = 0.0;
      delta_q[2] = 0.0;
      delta_q[3] = 0.5 * state->q[0];
      for (int i{0}; i < 3; i++) {
        state->P[10 * i] = R_position1[3 * i];
        state->P[10 * i + 1] = R_position1[3 * i + 1];
        state->P[10 * i + 2] = R_position1[3 * i + 2];
      }
      for (int i{0}; i < 4; i++) {
        int j;
        j = 10 * (i + 3);
        state->P[j + 3] = delta_q[0] * omega_norm * delta_q[i];
        yaw_den = 0.0 * omega_norm * delta_q[i];
        state->P[j + 4] = yaw_den;
        state->P[j + 5] = yaw_den;
        state->P[j + 6] = delta_q[3] * omega_norm * delta_q[i];
      }
      //  IMU sensor -> body rotation
      //
      //  Inputs:
      //    x_sensor_body = sensor x-axis expressed in body coordinates
      //    y_sensor_body = sensor y-axis expressed in body coordinates
      //    z_sensor_body = sensor z-axis expressed in body coordinates
      //
      //  Output:
      //    R_body_sensor = rotation matrix Sensor -> Body
      yaw_den = 3.3121686421112381E-170;
      sinPhi = 3.3121686421112381E-170;
      omega_norm = 3.3121686421112381E-170;
      absxk = std::abs(IMU->mounting.x_sensor_body[0]);
      if (absxk > 3.3121686421112381E-170) {
        unnamed_idx_0_tmp = 1.0;
        yaw_den = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        unnamed_idx_0_tmp = t * t;
      }
      absxk = std::abs(IMU->mounting.y_sensor_body[0]);
      if (absxk > 3.3121686421112381E-170) {
        theta = 1.0;
        sinPhi = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        theta = t * t;
      }
      absxk = std::abs(IMU->mounting.z_sensor_body[0]);
      if (absxk > 3.3121686421112381E-170) {
        psi_mid = 1.0;
        omega_norm = absxk;
      } else {
        t = absxk / 3.3121686421112381E-170;
        psi_mid = t * t;
      }
      absxk = std::abs(IMU->mounting.x_sensor_body[1]);
      if (absxk > yaw_den) {
        t = yaw_den / absxk;
        unnamed_idx_0_tmp = unnamed_idx_0_tmp * t * t + 1.0;
        yaw_den = absxk;
      } else {
        t = absxk / yaw_den;
        unnamed_idx_0_tmp += t * t;
      }
      absxk = std::abs(IMU->mounting.y_sensor_body[1]);
      if (absxk > sinPhi) {
        t = sinPhi / absxk;
        theta = theta * t * t + 1.0;
        sinPhi = absxk;
      } else {
        t = absxk / sinPhi;
        theta += t * t;
      }
      absxk = std::abs(IMU->mounting.z_sensor_body[1]);
      if (absxk > omega_norm) {
        t = omega_norm / absxk;
        psi_mid = psi_mid * t * t + 1.0;
        omega_norm = absxk;
      } else {
        t = absxk / omega_norm;
        psi_mid += t * t;
      }
      absxk = std::abs(IMU->mounting.x_sensor_body[2]);
      if (absxk > yaw_den) {
        t = yaw_den / absxk;
        unnamed_idx_0_tmp = unnamed_idx_0_tmp * t * t + 1.0;
        yaw_den = absxk;
      } else {
        t = absxk / yaw_den;
        unnamed_idx_0_tmp += t * t;
      }
      absxk = std::abs(IMU->mounting.y_sensor_body[2]);
      if (absxk > sinPhi) {
        t = sinPhi / absxk;
        theta = theta * t * t + 1.0;
        sinPhi = absxk;
      } else {
        t = absxk / sinPhi;
        theta += t * t;
      }
      absxk = std::abs(IMU->mounting.z_sensor_body[2]);
      if (absxk > omega_norm) {
        t = omega_norm / absxk;
        psi_mid = psi_mid * t * t + 1.0;
        omega_norm = absxk;
      } else {
        t = absxk / omega_norm;
        psi_mid += t * t;
      }
      unnamed_idx_0_tmp = yaw_den * std::sqrt(unnamed_idx_0_tmp);
      theta = sinPhi * std::sqrt(theta);
      psi_mid = omega_norm * std::sqrt(psi_mid);
      R_body_sensor[0] = IMU->mounting.x_sensor_body[0] / unnamed_idx_0_tmp;
      R_body_sensor[3] = IMU->mounting.y_sensor_body[0] / theta;
      R_body_sensor[6] = IMU->mounting.z_sensor_body[0] / psi_mid;
      R_body_sensor[1] = IMU->mounting.x_sensor_body[1] / unnamed_idx_0_tmp;
      R_body_sensor[4] = IMU->mounting.y_sensor_body[1] / theta;
      R_body_sensor[7] = IMU->mounting.z_sensor_body[1] / psi_mid;
      R_body_sensor[2] = IMU->mounting.x_sensor_body[2] / unnamed_idx_0_tmp;
      R_body_sensor[5] = IMU->mounting.y_sensor_body[2] / theta;
      R_body_sensor[8] = IMU->mounting.z_sensor_body[2] / psi_mid;
      prev_timestamp = IMU->timestamp;
      prev_velocity = Vehicle->velocity;
    }
  } else {
    double F[100];
    double P_posterior[100];
    double b_F[100];
    double absxk;
    double dt;
    double omega_norm;
    double psi_mid;
    double q_idx_0;
    double q_idx_2;
    double q_idx_3;
    double sinPhi;
    double t;
    double theta;
    double unnamed_idx_0_tmp;
    double yaw_den;
    int P_qb_prior_tmp_tmp;
    int R_measurement_tmp;
    int R_position1_tmp;
    int j;
    //  =========================================================
    //  PREDICTION BY IMU GYROSCOPE + BICYCLE MODEL
    //  ==========================================================
    if (IMU->valid) {
      double b_G_gyro[30];
      boolean_T valid;
      // disp(">>> ABOUT TO CALL prediction_step");
      dt = (IMU->timestamp - prev_timestamp) / 1000.0;
      // [sec]
      prev_timestamp = IMU->timestamp;
      // ms
      //  IMU gives angular velocity in SENSOR frame
      //  Transform the same angular velocity into BODY frame
      //
      //  omega_sensor = angular velocity measured in sensor frame
      //  omega_body   = same angular velocity expressed in body frame
      yaw_den = IMU->gyro[0];
      absxk = IMU->gyro[1];
      t = IMU->gyro[2];
      for (int i{0}; i < 3; i++) {
        omega[i] = (R_body_sensor[i] * yaw_den + R_body_sensor[i + 3] * absxk) +
                   R_body_sensor[i + 6] * t;
      }
      //  prediction_step expects gyro measurement in BODY frame
      for (int i{0}; i < 3; i++) {
        IMU->gyro[i] = omega[i];
        for (j = 0; j < 3; j++) {
          R_position1_tmp = i + 3 * j;
          R_position1[j + 3 * i] = R_body_sensor[R_position1_tmp];
          H_q[R_position1_tmp] =
              (R_body_sensor[i] * IMU->gyro_covariance[3 * j] +
               R_body_sensor[i + 3] * IMU->gyro_covariance[3 * j + 1]) +
              R_body_sensor[i + 6] * IMU->gyro_covariance[3 * j + 2];
        }
      }
      //
      //  omega_sensor = angular velocity measured in sensor frame
      //  omega_body   = same angular velocity expressed in body frame
      for (int i{0}; i < 3; i++) {
        yaw_den = H_q[i];
        absxk = H_q[i + 3];
        t = H_q[i + 6];
        psi_mid = 0.0;
        for (j = 0; j < 3; j++) {
          R_measurement_tmp = i + 3 * j;
          IMU->gyro_covariance[R_measurement_tmp] =
              (yaw_den * R_position1[3 * j] + absxk * R_position1[3 * j + 1]) +
              t * R_position1[3 * j + 2];
          psi_mid += R_body_sensor[R_measurement_tmp] * IMU->accel[j];
        }
        omega[i] = psi_mid;
      }
      for (int i{0}; i < 3; i++) {
        IMU->accel[i] = omega[i];
        yaw_den = R_body_sensor[i];
        absxk = R_body_sensor[i + 3];
        t = R_body_sensor[i + 6];
        for (j = 0; j < 3; j++) {
          H_q[i + 3 * j] = (yaw_den * IMU->accel_covariance[3 * j] +
                            absxk * IMU->accel_covariance[3 * j + 1]) +
                           t * IMU->accel_covariance[3 * j + 2];
        }
      }
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
      // DEBUG
      // disp("prediction step is invoked");
      //  =========================================================
      //  1. SAVE PREVIOUS STATE
      //  ==========================================================
      //  =========================================================
      //  2. REMOVE ESTIMATED GYRO BIAS
      //  ==========================================================
      for (R_position1_tmp = 0; R_position1_tmp < 3; R_position1_tmp++) {
        yaw_den = H_q[R_position1_tmp];
        absxk = H_q[R_position1_tmp + 3];
        t = H_q[R_position1_tmp + 6];
        for (int i{0}; i < 3; i++) {
          IMU->accel_covariance[R_position1_tmp + 3 * i] =
              (yaw_den * R_position1[3 * i] + absxk * R_position1[3 * i + 1]) +
              t * R_position1[3 * i + 2];
        }
        omega[R_position1_tmp] =
            IMU->gyro[R_position1_tmp] - state->bg[R_position1_tmp];
      }
      //  best estimate for angular veolcity
      //  =========================================================
      //  3. CALCULATE ROTATION DURING dt
      //  ==========================================================
      omega_norm = std::sqrt((omega[0] * omega[0] + omega[1] * omega[1]) +
                             omega[2] * omega[2]);
      if (omega_norm > 1.0E-8) {
        //  Total rotation angle during dt (during dt ang vel is supposed to be
        //  constant)
        theta = omega_norm * dt;
        //  Rotation axis
        //  Incremental quaternion : rotation of angle theta around u axis
        psi_mid = std::sin(theta / 2.0);
        delta_q[0] = std::cos(theta / 2.0);
        delta_q[1] = omega[0] / omega_norm * psi_mid;
        delta_q[2] = omega[1] / omega_norm * psi_mid;
        delta_q[3] = omega[2] / omega_norm * psi_mid;
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
      q_prior[0] = ((state->q[0] * delta_q[0] - state->q[1] * delta_q[1]) -
                    state->q[2] * delta_q[2]) -
                   state->q[3] * delta_q[3];
      q_prior[1] = ((state->q[0] * delta_q[1] + delta_q[0] * state->q[1]) +
                    state->q[2] * delta_q[3]) -
                   delta_q[2] * state->q[3];
      q_prior[2] = ((state->q[0] * delta_q[2] - state->q[1] * delta_q[3]) +
                    delta_q[0] * state->q[2]) +
                   delta_q[1] * state->q[3];
      q_prior[3] = ((state->q[0] * delta_q[3] + state->q[1] * delta_q[2]) -
                    delta_q[1] * state->q[2]) +
                   delta_q[0] * state->q[3];
      //  =========================================================
      //  5. NORMALIZE QUATERNION : bcs of numerical instability and valid
      //  rotation must satistfies  ||q|| = 1
      //  ==========================================================
      omega_norm =
          std::sqrt(((q_prior[0] * q_prior[0] + q_prior[1] * q_prior[1]) +
                     q_prior[2] * q_prior[2]) +
                    q_prior[3] * q_prior[3]);
      q_prior[0] /= omega_norm;
      q_prior[1] /= omega_norm;
      q_prior[2] /= omega_norm;
      q_prior[3] /= omega_norm;
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
      theta = 2.0 * (state->q[0] * state->q[3] + state->q[1] * state->q[2]);
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
      q_idx_0 = std::tan(Vehicle->steering_angle);
      psi_mid = rt_atan2d_snf(theta, yaw_den) +
                0.5 * (Vehicle->velocity / Vehicle->wheelbase * q_idx_0) * dt;
      //  ---------------------------------------------------------
      //  6.4 Predict ENU position
      //  ---------------------------------------------------------
      //
      //  E_k = E_(k-1) + v*cos(psi_mid)*dt
      //
      //  N_k = N_(k-1) + v*sin(psi_mid)*dt
      //
      //  U_k = U_(k-1)
      absxk = Vehicle->velocity * std::cos(psi_mid) * dt;
      state->p[0] += absxk;
      sinPhi = std::sin(psi_mid);
      state->p[1] += Vehicle->velocity * sinPhi * dt;
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
      for (int i{0}; i < 100; i++) {
        F[i] = b_I[i];
      }
      //  ---------------------------------------------------------
      //  8.1 POSITION -> POSITION
      //  ---------------------------------------------------------
      //
      //  p_k contains p_(k-1), therefore:
      //
      //  dp_k / dp_(k-1) = I
      for (int i{0}; i < 3; i++) {
        F[10 * i] = iv[3 * i];
        F[10 * i + 1] = iv[3 * i + 1];
        F[10 * i + 2] = iv[3 * i + 2];
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
      omega_norm = theta * theta + yaw_den * yaw_den;
      //  Derivatives of A with respect to quaternion
      //  Derivatives of B with respect to quaternion
      //  Derivative of atan2(A,B):
      //
      //  dpsi/dq =
      //
      //  (B*dA/dq - A*dB/dq)
      //  ---------------------
      //        A^2 + B^2
      delta_q[0] = (yaw_den * (2.0 * state->q[3]) - theta * 0.0) / omega_norm;
      delta_q[1] = (yaw_den * (2.0 * state->q[2]) - theta * 0.0) / omega_norm;
      delta_q[2] =
          (yaw_den * (2.0 * state->q[1]) - theta * (-4.0 * state->q[2])) /
          omega_norm;
      delta_q[3] =
          (yaw_den * (2.0 * state->q[0]) - theta * (-4.0 * state->q[3])) /
          omega_norm;
      //  Position derivative with respect to yaw
      //
      //  E = E_prev + v*cos(psi_mid)*dt
      //
      //  N = N_prev + v*sin(psi_mid)*dt
      psi_mid = -Vehicle->velocity * sinPhi * dt;
      //  Chain rule:
      //
      //  dp/dq = dp/dpsi * dpsi/dq
      F[30] = psi_mid * delta_q[0];
      F[31] = absxk * delta_q[0];
      F[32] = 0.0;
      F[40] = psi_mid * delta_q[1];
      F[41] = absxk * delta_q[1];
      F[42] = 0.0;
      F[50] = psi_mid * delta_q[2];
      F[51] = absxk * delta_q[2];
      F[52] = 0.0;
      F[60] = psi_mid * delta_q[3];
      F[61] = absxk * delta_q[3];
      F[62] = 0.0;
      //  ---------------------------------------------------------
      //  8.3 QUATERNION -> QUATERNION
      //  ---------------------------------------------------------
      sinPhi = 0.0 * dt;
      omega_norm = 0.5 * -omega[0] * dt;
      theta = 0.5 * -omega[1] * dt;
      psi_mid = 0.5 * -omega[2] * dt;
      absxk = 0.5 * omega[0] * dt;
      t = 0.5 * omega[2] * dt;
      unnamed_idx_0_tmp = 0.5 * omega[1] * dt;
      F[33] = sinPhi + 1.0;
      F[34] = absxk;
      F[35] = unnamed_idx_0_tmp;
      F[36] = t;
      F[43] = omega_norm;
      F[44] = sinPhi + 1.0;
      F[45] = psi_mid;
      F[46] = unnamed_idx_0_tmp;
      F[53] = theta;
      F[54] = t;
      F[55] = sinPhi + 1.0;
      F[56] = omega_norm;
      F[63] = psi_mid;
      F[64] = theta;
      F[65] = absxk;
      F[66] = sinPhi + 1.0;
      //  ---------------------------------------------------------
      //  8.4 GYRO BIAS -> QUATERNION
      //  ---------------------------------------------------------
      sinPhi = -0.5 * -state->q[1] * dt;
      F[73] = sinPhi;
      omega_norm = -0.5 * -state->q[2] * dt;
      F[83] = omega_norm;
      theta = -0.5 * -state->q[3] * dt;
      F[93] = theta;
      psi_mid = -0.5 * state->q[0] * dt;
      F[74] = psi_mid;
      F[84] = theta;
      F[94] = -0.5 * state->q[2] * dt;
      F[75] = -0.5 * state->q[3] * dt;
      F[85] = psi_mid;
      F[95] = sinPhi;
      F[76] = omega_norm;
      F[86] = -0.5 * state->q[1] * dt;
      F[96] = psi_mid;
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
      sinPhi = 0.5 * -state->q[1] * dt;
      G_gyro[3] = sinPhi;
      omega_norm = 0.5 * -state->q[2] * dt;
      G_gyro[13] = omega_norm;
      theta = 0.5 * -state->q[3] * dt;
      G_gyro[23] = theta;
      psi_mid = 0.5 * state->q[0] * dt;
      G_gyro[4] = psi_mid;
      G_gyro[14] = theta;
      G_gyro[24] = 0.5 * state->q[2] * dt;
      G_gyro[5] = 0.5 * state->q[3] * dt;
      G_gyro[15] = psi_mid;
      G_gyro[25] = sinPhi;
      G_gyro[6] = omega_norm;
      G_gyro[16] = 0.5 * state->q[1] * dt;
      G_gyro[26] = psi_mid;
      //  Gyroscope contribution to prediction covariance
      //  =========================================================
      //  10. COVARIANCE PREDICTION
      //  ==========================================================
      for (int i{0}; i < 10; i++) {
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den += F[i + 10 * R_measurement_tmp] *
                       state->P[R_measurement_tmp + 10 * j];
          }
          b_F[i + 10 * j] = yaw_den;
        }
        yaw_den = G_gyro[i];
        absxk = G_gyro[i + 10];
        t = G_gyro[i + 20];
        for (j = 0; j < 3; j++) {
          b_G_gyro[i + 10 * j] = (yaw_den * IMU->gyro_covariance[3 * j] +
                                  absxk * IMU->gyro_covariance[3 * j + 1]) +
                                 t * IMU->gyro_covariance[3 * j + 2];
        }
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den +=
                b_F[i + 10 * R_measurement_tmp] * F[j + 10 * R_measurement_tmp];
          }
          P_posterior[i + 10 * j] = yaw_den;
        }
      }
      for (int i{0}; i < 10; i++) {
        yaw_den = b_G_gyro[i];
        absxk = b_G_gyro[i + 10];
        t = b_G_gyro[i + 20];
        for (j = 0; j < 10; j++) {
          F[i + 10 * j] = (yaw_den * G_gyro[j] + absxk * G_gyro[j + 10]) +
                          t * G_gyro[j + 20];
        }
      }
      for (int i{0}; i < 100; i++) {
        state->P[i] = (P_posterior[i] + F[i]) + state->Q[i];
      }
      //  =========================================================
      //  11. SAVE A PRIORI STATE
      //  ==========================================================
      //  =====================================================
      //  ACCELEROMETER GRAVITY CORRECTION
      //
      //  Applied only if:
      //
      //    - longitudinal acceleration is small
      //    - lateral acceleration is small
      //    - |norm(accel) - g| is small
      //
      //  This correction modifies ONLY:
      //
      //    state.q
      //    Pqq
      //    Pqb
      //    Pbq
      //
      //  =====================================================
      //
      //  Gravity-based accelerometer correction.
      //
      //  IMPORTANT:
      //
      //  This correction modifies ONLY:
      //
      //    state.q
      //
      //  and covariance blocks:
      //
      //    Pqq = P(4:7,4:7)
      //    Pqb = P(4:7,8:10)
      //    Pbq = P(8:10,4:7)
      //
      //  It does NOT modify:
      //
      //    state.p
      //    state.bg
      //
      //  Assumption:
      //  IMU.accel is expressed in BODY coordinates.
      //
      //  State ordering:
      //
      //  x =
      //  [
      //    p(3)
      //    q(4)
      //    bg(3)
      //  ]
      //  =========================================================
      //  1. A PRIORI QUATERNION
      //  =========================================================
      yaw_den = coder::b_norm(q_prior);
      state->q[0] = q_prior[0];
      q_prior[0] /= yaw_den;
      state->q[1] = q_prior[1];
      q_prior[1] /= yaw_den;
      state->q[2] = q_prior[2];
      q_prior[2] /= yaw_den;
      state->q[3] = q_prior[3];
      q_prior[3] /= yaw_den;
      //  =========================================================
      //  2. CALCULATE ACCELEROMETER INNOVATION
      //
      //  Also performs low-dynamics validity check.
      //  =========================================================
      //
      //  Calculates gravity-based accelerometer innovation.
      //
      //  All acceleration vectors used for the innovation are
      //  expressed in BODY coordinates.
      //
      //  Assumption:
      //
      //  Vehicle dynamic acceleration is sufficiently small so that
      //  the accelerometer measurement is dominated by gravity.
      //  =========================================================
      //  CONSTANTS
      //  =========================================================
      //  [m/s^2]
      //  =========================================================
      //  DEFAULT OUTPUTS
      //  =========================================================
      omega[0] = 0.0;
      omega[1] = 0.0;
      omega[2] = 0.0;
      valid = false;
      //  =========================================================
      //  BASIC CHECKS
      //  =========================================================
      if ((!(dt <= 0.0)) && (!(Vehicle->wheelbase <= 0.0))) {
        //  =========================================================
        //  1. LONGITUDINAL ACCELERATION
        //
        //                  v(k) - v(k-1)
        //  a_long = -----------------------------
        //                        dt
        //  =========================================================
        theta = (Vehicle->velocity - prev_velocity) / dt;
        //  =========================================================
        //  2. LATERAL ACCELERATION FROM BICYCLE MODEL
        //
        //  yaw_rate =
        //
        //            v
        //           --- tan(delta)
        //            L
        //
        //  and:
        //
        //  a_lat = v * yaw_rate
        //
        //  therefore:
        //
        //              v^2
        //  a_lat = ----------- tan(delta)
        //               L
        //  =========================================================
        omega_norm = Vehicle->velocity * Vehicle->velocity /
                     Vehicle->wheelbase * q_idx_0;
        //  =========================================================
        //  3. ESTIMATED DYNAMIC HORIZONTAL ACCELERATION
        //  =========================================================
        //  4. FIRST LOW-DYNAMICS CONDITION
        //  =========================================================
        if (!(std::sqrt(theta * theta + omega_norm * omega_norm) >= 0.5)) {
          //  =========================================================
          //  5. ACCELEROMETER MAGNITUDE CHECK
          //
          //  norm([ax ay az]') should be close to g
          //  if gravity dominates.
          //  =========================================================
          yaw_den = 3.3121686421112381E-170;
          absxk = std::abs(IMU->accel[0]);
          if (absxk > 3.3121686421112381E-170) {
            unnamed_idx_0_tmp = 1.0;
            yaw_den = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            unnamed_idx_0_tmp = t * t;
          }
          absxk = std::abs(IMU->accel[1]);
          if (absxk > yaw_den) {
            t = yaw_den / absxk;
            unnamed_idx_0_tmp = unnamed_idx_0_tmp * t * t + 1.0;
            yaw_den = absxk;
          } else {
            t = absxk / yaw_den;
            unnamed_idx_0_tmp += t * t;
          }
          absxk = std::abs(IMU->accel[2]);
          if (absxk > yaw_den) {
            t = yaw_den / absxk;
            unnamed_idx_0_tmp = unnamed_idx_0_tmp * t * t + 1.0;
            yaw_den = absxk;
          } else {
            t = absxk / yaw_den;
            unnamed_idx_0_tmp += t * t;
          }
          unnamed_idx_0_tmp = yaw_den * std::sqrt(unnamed_idx_0_tmp);
          if (!(std::abs(unnamed_idx_0_tmp - 9.81) >= 0.5)) {
            //  =========================================================
            //  6. NORMALIZE A PRIORI QUATERNION
            //
            //  q describes BODY -> ENU
            //  =========================================================
            yaw_den = coder::b_norm(q_prior);
            q_idx_0 = q_prior[0] / yaw_den;
            dt = q_prior[1] / yaw_den;
            q_idx_2 = q_prior[2] / yaw_den;
            q_idx_3 = q_prior[3] / yaw_den;
            //  =========================================================
            //  7. BODY -> ENU ROTATION MATRIX
            //  =========================================================
            //  8. ENU -> BODY ROTATION
            //  =========================================================
            //  9. GRAVITY-RELATED SPECIFIC FORCE IN ENU
            //
            //  Actual gravitational acceleration is:
            //
            //      [0]
            //  g = [0]
            //      [-g]
            //
            //  A stationary accelerometer measures opposite gravity:
            //
            //      [0]
            //      [+g]
            //  =========================================================
            //  10. PREDICT ACCELEROMETER MEASUREMENT IN BODY FRAME
            //  =========================================================
            //  11. INNOVATION
            //
            //  measured - predicted
            //
            //  BOTH are in BODY coordinates.
            //  =========================================================
            omega_norm = q_idx_3 * q_idx_3;
            theta = q_idx_2 * q_idx_2;
            H_q[0] = 1.0 - 2.0 * (theta + omega_norm);
            psi_mid = dt * q_idx_2;
            absxk = q_idx_0 * q_idx_3;
            H_q[1] = 2.0 * (psi_mid - absxk);
            t = dt * q_idx_3;
            yaw_den = q_idx_0 * q_idx_2;
            H_q[2] = 2.0 * (t + yaw_den);
            H_q[3] = 2.0 * (psi_mid + absxk);
            psi_mid = dt * dt;
            H_q[4] = 1.0 - 2.0 * (psi_mid + omega_norm);
            omega_norm = q_idx_2 * q_idx_3;
            absxk = q_idx_0 * dt;
            H_q[5] = 2.0 * (omega_norm - absxk);
            H_q[6] = 2.0 * (t - yaw_den);
            H_q[7] = 2.0 * (omega_norm + absxk);
            H_q[8] = 1.0 - 2.0 * (psi_mid + theta);
            for (int i{0}; i < 3; i++) {
              omega[i] = IMU->accel[i] - ((H_q[i] * 0.0 + H_q[i + 3] * 0.0) +
                                          H_q[i + 6] * 9.81);
            }
            //  =========================================================
            //  MEASUREMENT IS ACCEPTED
            //  =========================================================
            valid = true;
          }
        }
      }
      //  =========================================================
      //  3. IF GRAVITY MODEL IS NOT VALID:
      //     SKIP THIS CORRECTION
      //  =========================================================
      if (valid) {
        double A_q[16];
        double b_A_q[16];
        double c_A_q[16];
        double K_q[12];
        double P_qb_prior[12];
        double b_H_q[12];
        double b_state[12];
        double c_H_q[12];
        signed char c_I[16];
        //  =========================================================
        //  4. EXTRACT RELEVANT COVARIANCE BLOCKS
        //  =========================================================
        //  =========================================================
        //  5. ACCELEROMETER MEASUREMENT JACOBIAN
        //
        //  H_q:
        //
        //       d(acc_pred_body)
        //       ----------------
        //              dq
        //
        //  Dimension: 3x4
        //  =========================================================
        //
        //  Accelerometer measurement Jacobian with respect
        //  to quaternion only.
        //
        //  Quaternion:
        //
        //  q = [qw qx qy qz]'
        //
        //  Predicted accelerometer:
        //
        //  a_pred_body =
        //
        //      R_BE(q) * [0; 0; g]
        //
        //  H_q dimension:
        //
        //      3 x 4
        //
        //  because:
        //
        //  3 accelerometer measurements
        //  4 quaternion state components
        //  =========================================================
        //  NORMALIZE QUATERNION
        //  =========================================================
        yaw_den = coder::b_norm(q_prior);
        q_idx_0 = q_prior[0] / yaw_den;
        dt = q_prior[1] / yaw_den;
        q_idx_2 = q_prior[2] / yaw_den;
        //  =========================================================
        //  Predicted acceleration:
        //
        //  ax = 2g(qx*qz - qw*qy)
        //
        //  ay = 2g(qy*qz + qw*qx)
        //
        //  az = g(1 - 2*qx^2 - 2*qy^2)
        //
        //  H_q =
        //
        //       d[a_x a_y a_z]'
        //       ----------------
        //       d[qw qx qy qz]
        //  =========================================================
        b_H_q[0] = 9.81 * (-2.0 * q_idx_2);
        omega_norm = 9.81 * (2.0 * (q_prior[3] / yaw_den));
        b_H_q[3] = omega_norm;
        b_H_q[6] = 9.81 * (-2.0 * q_idx_0);
        theta = 9.81 * (2.0 * dt);
        b_H_q[9] = theta;
        b_H_q[1] = theta;
        b_H_q[4] = 9.81 * (2.0 * q_idx_0);
        b_H_q[7] = omega_norm;
        b_H_q[10] = 9.81 * (2.0 * q_idx_2);
        b_H_q[2] = 0.0;
        b_H_q[5] = 9.81 * (-4.0 * dt);
        b_H_q[8] = 9.81 * (-4.0 * q_idx_2);
        b_H_q[11] = 0.0;
        //  =========================================================
        //  6. ACCELEROMETER MEASUREMENT COVARIANCE
        //
        //  If IMU.accel_covariance already contains VARIANCES,
        //  DO NOT square them again.
        //  =========================================================
        //  7. INNOVATION COVARIANCE
        //
        //  Since this correction operates only on quaternion:
        //
        //  S = Hq Pqq Hq' + Racc
        //
        //  Dimension: 3x3
        //  =========================================================
        //  =========================================================
        //  8. QUATERNION KALMAN GAIN
        //
        //  Dimension:
        //
        //  Pqq : 4x4
        //  Hq' : 4x3
        //
        //  Kq  : 4x3
        //  =========================================================
        for (int i{0}; i < 3; i++) {
          R_position1_tmp = 10 * (i + 7);
          P_qb_prior_tmp_tmp = i << 2;
          P_qb_prior[P_qb_prior_tmp_tmp] = state->P[R_position1_tmp + 3];
          P_qb_prior[P_qb_prior_tmp_tmp + 1] = state->P[R_position1_tmp + 4];
          P_qb_prior[P_qb_prior_tmp_tmp + 2] = state->P[R_position1_tmp + 5];
          P_qb_prior[P_qb_prior_tmp_tmp + 3] = state->P[R_position1_tmp + 6];
          for (j = 0; j < 4; j++) {
            R_position1_tmp = i + 3 * j;
            K_q[j + P_qb_prior_tmp_tmp] = b_H_q[R_position1_tmp];
            R_measurement_tmp = 10 * (j + 3);
            c_H_q[R_position1_tmp] =
                ((b_H_q[i] * state->P[R_measurement_tmp + 3] +
                  b_H_q[i + 3] * state->P[R_measurement_tmp + 4]) +
                 b_H_q[i + 6] * state->P[R_measurement_tmp + 5]) +
                b_H_q[i + 9] * state->P[R_measurement_tmp + 6];
          }
        }
        for (int i{0}; i < 4; i++) {
          yaw_den = state->P[i + 33];
          absxk = state->P[i + 43];
          t = state->P[i + 53];
          psi_mid = state->P[i + 63];
          for (j = 0; j < 3; j++) {
            R_measurement_tmp = j << 2;
            b_state[i + R_measurement_tmp] =
                ((yaw_den * K_q[R_measurement_tmp] +
                  absxk * K_q[R_measurement_tmp + 1]) +
                 t * K_q[R_measurement_tmp + 2]) +
                psi_mid * K_q[R_measurement_tmp + 3];
          }
        }
        for (int i{0}; i < 3; i++) {
          yaw_den = c_H_q[i];
          absxk = c_H_q[i + 3];
          t = c_H_q[i + 6];
          psi_mid = c_H_q[i + 9];
          for (j = 0; j < 3; j++) {
            R_measurement_tmp = j << 2;
            R_position1_tmp = i + 3 * j;
            H_q[R_position1_tmp] = (((yaw_den * K_q[R_measurement_tmp] +
                                      absxk * K_q[R_measurement_tmp + 1]) +
                                     t * K_q[R_measurement_tmp + 2]) +
                                    psi_mid * K_q[R_measurement_tmp + 3]) +
                                   IMU->accel_covariance[R_position1_tmp];
          }
        }
        coder::internal::mrdiv(b_state, H_q, K_q);
        //  =========================================================
        //  9. QUATERNION CORRECTION
        //  =========================================================
        yaw_den = omega[0];
        absxk = omega[1];
        t = omega[2];
        for (int i{0}; i < 4; i++) {
          delta_q[i] = q_prior[i] + ((K_q[i] * yaw_den + K_q[i + 4] * absxk) +
                                     K_q[i + 8] * t);
        }
        //  =========================================================
        //  10. NORMALIZE QUATERNION
        //  =========================================================
        yaw_den = coder::b_norm(delta_q);
        state->q[0] = delta_q[0] / yaw_den;
        state->q[1] = delta_q[1] / yaw_den;
        state->q[2] = delta_q[2] / yaw_den;
        state->q[3] = delta_q[3] / yaw_den;
        //  =========================================================
        //  11. UPDATE QUATERNION COVARIANCE
        //
        //  Joseph form on quaternion subsystem only.
        //  =========================================================
        for (int i{0}; i < 16; i++) {
          c_I[i] = 0;
        }
        c_I[0] = 1;
        c_I[5] = 1;
        c_I[10] = 1;
        c_I[15] = 1;
        for (int i{0}; i < 4; i++) {
          yaw_den = K_q[i];
          absxk = K_q[i + 4];
          t = K_q[i + 8];
          for (j = 0; j < 4; j++) {
            R_position1_tmp = i + (j << 2);
            A_q[R_position1_tmp] =
                static_cast<double>(c_I[R_position1_tmp]) -
                ((yaw_den * b_H_q[3 * j] + absxk * b_H_q[3 * j + 1]) +
                 t * b_H_q[3 * j + 2]);
          }
          psi_mid = A_q[i];
          sinPhi = A_q[i + 4];
          omega_norm = A_q[i + 8];
          theta = A_q[i + 12];
          for (j = 0; j < 4; j++) {
            R_measurement_tmp = 10 * (j + 3);
            b_A_q[i + (j << 2)] =
                ((psi_mid * state->P[R_measurement_tmp + 3] +
                  sinPhi * state->P[R_measurement_tmp + 4]) +
                 omega_norm * state->P[R_measurement_tmp + 5]) +
                theta * state->P[R_measurement_tmp + 6];
          }
          for (j = 0; j < 3; j++) {
            b_state[i + (j << 2)] = (yaw_den * IMU->accel_covariance[3 * j] +
                                     absxk * IMU->accel_covariance[3 * j + 1]) +
                                    t * IMU->accel_covariance[3 * j + 2];
          }
        }
        for (int i{0}; i < 4; i++) {
          yaw_den = b_A_q[i];
          absxk = b_A_q[i + 4];
          t = b_A_q[i + 8];
          psi_mid = b_A_q[i + 12];
          for (j = 0; j < 4; j++) {
            c_A_q[i + (j << 2)] =
                ((yaw_den * A_q[j] + absxk * A_q[j + 4]) + t * A_q[j + 8]) +
                psi_mid * A_q[j + 12];
          }
          yaw_den = b_state[i];
          absxk = b_state[i + 4];
          t = b_state[i + 8];
          for (j = 0; j < 4; j++) {
            b_A_q[i + (j << 2)] =
                (yaw_den * K_q[j] + absxk * K_q[j + 4]) + t * K_q[j + 8];
          }
        }
        //  =========================================================
        //  12. UPDATE QUATERNION <-> GYRO BIAS CROSS COVARIANCE
        //
        //  Bias itself is NOT corrected.
        //
        //  We only preserve/update its statistical relation with q.
        //  =========================================================
        for (int i{0}; i < 4; i++) {
          j = i << 2;
          R_measurement_tmp = 10 * (i + 3);
          state->P[R_measurement_tmp + 3] = c_A_q[j] + b_A_q[j];
          state->P[R_measurement_tmp + 4] = c_A_q[j + 1] + b_A_q[j + 1];
          state->P[R_measurement_tmp + 5] = c_A_q[j + 2] + b_A_q[j + 2];
          state->P[R_measurement_tmp + 6] = c_A_q[j + 3] + b_A_q[j + 3];
          yaw_den = A_q[i];
          absxk = A_q[i + 4];
          t = A_q[i + 8];
          psi_mid = A_q[i + 12];
          for (j = 0; j < 3; j++) {
            R_measurement_tmp = j << 2;
            K_q[i + R_measurement_tmp] =
                ((yaw_den * P_qb_prior[R_measurement_tmp] +
                  absxk * P_qb_prior[R_measurement_tmp + 1]) +
                 t * P_qb_prior[R_measurement_tmp + 2]) +
                psi_mid * P_qb_prior[R_measurement_tmp + 3];
          }
        }
        //  =========================================================
        //  13. WRITE BACK ONLY DESIRED STATE/COVARIANCE PARTS
        //  =========================================================
        //  Quaternion covariance
        //  Quaternion -> bias cross covariance
        for (int i{0}; i < 3; i++) {
          j = i << 2;
          R_measurement_tmp = 10 * (i + 7);
          state->P[R_measurement_tmp + 3] = K_q[j];
          state->P[R_measurement_tmp + 4] = K_q[j + 1];
          state->P[R_measurement_tmp + 5] = K_q[j + 2];
          state->P[R_measurement_tmp + 6] = K_q[j + 3];
        }
        //  Bias -> quaternion cross covariance
        //  Keep P symmetric.
        for (int i{0}; i < 4; i++) {
          j = 10 * (i + 3);
          state->P[j + 7] = K_q[i];
          state->P[j + 8] = K_q[i + 4];
          state->P[j + 9] = K_q[i + 8];
        }
        //  =========================================================
        //  Everything else in state.P remains untouched.
        //  =========================================================
      }
      //     %% -----------------------------------------------------
      //  Store velocity for NEXT acceleration estimate
      //
      //  IMPORTANT:
      //  Do this after accelerometer correction because the
      //  current correction needs v(k-1).
      //  ------------------------------------------------------
      prev_velocity = Vehicle->velocity;
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
      sinPhi = std::sin(GNSS->lla1[0]);
      t = 6.378137E+6 /
          std::sqrt(1.0 - 0.0066943799901413165 * sinPhi * sinPhi);
      //  =========================================================
      //  4. LLA -> ECEF
      //  ==========================================================
      //  =========================================================
      //  5. OUTPUT VECTOR
      //  ==========================================================
      psi_mid = (t + GNSS->lla1[2]) * std::cos(GNSS->lla1[0]);
      absxk = psi_mid * std::cos(GNSS->lla1[1]);
      omega_norm = psi_mid * std::sin(GNSS->lla1[1]);
      sinPhi *= t * 0.99330562000985867 + GNSS->lla1[2];
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
      yaw_den = coder::b_norm(state->q);
      for (int i{0}; i < 4; i++) {
        delta_q[i] = ((T_enu_ecef[i] * absxk + T_enu_ecef[i + 4] * omega_norm) +
                      T_enu_ecef[i + 8] * sinPhi) +
                     T_enu_ecef[i + 12];
        q_prior[i] = state->q[i] / yaw_den;
      }
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
      yaw_den = coder::b_norm(q_prior);
      q_idx_0 = q_prior[0] / yaw_den;
      dt = q_prior[1] / yaw_den;
      q_idx_2 = q_prior[2] / yaw_den;
      q_idx_3 = q_prior[3] / yaw_den;
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
      //  DEBUG
      // {
      // disp("b_measured_enu:");
      // disp(b_measured_enu);
      //
      // disp("b_predicted_enu:");
      // disp(b_predicted_enu);
      //
      // disp("baseline norm measured:");
      // disp(norm(b_measured_enu));
      //
      // disp("baseline norm predicted:");
      // disp(norm(b_predicted_enu));
      //
      // disp("baseline_body:");
      // disp(GNSS.baseline_body);
      // }
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
      for (int i{0}; i < 3; i++) {
        H[6 * i] = iv[3 * i];
        H[6 * i + 1] = iv[3 * i + 1];
        H[6 * i + 2] = iv[3 * i + 2];
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
      H[21] = 2.0 * (-q_idx_3 * GNSS->baseline_body[1] + omega_norm);
      theta = GNSS->baseline_body[2] * q_idx_3;
      psi_mid = GNSS->baseline_body[1] * q_idx_2;
      H[27] = 2.0 * (psi_mid + theta);
      absxk = q_idx_0 * GNSS->baseline_body[2];
      t = dt * GNSS->baseline_body[1];
      H[33] = 2.0 * ((-2.0 * q_idx_2 * GNSS->baseline_body[0] + t) + absxk);
      yaw_den = dt * GNSS->baseline_body[2];
      unnamed_idx_0_tmp = q_idx_0 * GNSS->baseline_body[1];
      H[39] =
          2.0 * ((-2.0 * q_idx_3 * GNSS->baseline_body[0] - unnamed_idx_0_tmp) +
                 yaw_den);
      sinPhi = GNSS->baseline_body[0] * q_idx_3;
      H[22] = 2.0 * (sinPhi - yaw_den);
      H[28] = 2.0 * ((GNSS->baseline_body[0] * q_idx_2 -
                      2.0 * dt * GNSS->baseline_body[1]) -
                     absxk);
      absxk = GNSS->baseline_body[0] * dt;
      H[34] = 2.0 * (absxk + theta);
      H[40] = 2.0 * ((q_idx_0 * GNSS->baseline_body[0] -
                      2.0 * q_idx_3 * GNSS->baseline_body[1]) +
                     omega_norm);
      H[23] = 2.0 * (-q_idx_2 * GNSS->baseline_body[0] + t);
      H[29] = 2.0 * ((sinPhi + unnamed_idx_0_tmp) -
                     2.0 * dt * GNSS->baseline_body[2]);
      H[35] = 2.0 * ((-q_idx_0 * GNSS->baseline_body[0] +
                      GNSS->baseline_body[1] * q_idx_3) -
                     2.0 * q_idx_2 * GNSS->baseline_body[2]);
      H[41] = 2.0 * (absxk + psi_mid);
      //  =========================================================
      //  7. MEASUREMENT COVARIANCE
      //  =========================================================
      absxk = GNSS->hAcc1 * GNSS->hAcc1;
      std::memset(&R_position1[0], 0, 9U * sizeof(double));
      R_position1[0] = absxk;
      R_position1[4] = absxk;
      R_position1[8] = GNSS->vAcc1 * GNSS->vAcc1;
      //  This covariance corresponds to measurement vector:
      //
      //  z =
      //
      //  [ p1
      //    p2 - p1 ]
      //
      //  assuming receiver errors are independent.
      absxk = GNSS->hAcc2 * GNSS->hAcc2;
      omega[0] = absxk;
      omega[1] = absxk;
      omega[2] = GNSS->vAcc2 * GNSS->vAcc2;
      std::memset(&H_q[0], 0, 9U * sizeof(double));
      for (j = 0; j < 3; j++) {
        H_q[j + 3 * j] = omega[j];
        yaw_den = R_position1[3 * j];
        R_measurement[6 * j] = yaw_den;
        R_position1_tmp = 6 * (j + 3);
        R_measurement[R_position1_tmp] = -yaw_den;
        R_measurement[6 * j + 3] = -yaw_den;
        P_qb_prior_tmp_tmp = 3 * j + 1;
        absxk = R_position1[P_qb_prior_tmp_tmp];
        R_measurement[6 * j + 1] = absxk;
        R_measurement[R_position1_tmp + 1] = -absxk;
        R_measurement[6 * j + 4] = -absxk;
        R_measurement_tmp = 3 * j + 2;
        t = R_position1[R_measurement_tmp];
        R_measurement[6 * j + 2] = t;
        R_measurement[R_position1_tmp + 2] = -t;
        R_measurement[6 * j + 5] = -t;
        R_measurement[R_position1_tmp + 3] = yaw_den + H_q[3 * j];
        R_measurement[R_position1_tmp + 4] = absxk + H_q[P_qb_prior_tmp_tmp];
        R_measurement[R_position1_tmp + 5] = t + H_q[R_measurement_tmp];
      }
      //  =========================================================
      //  8. INNOVATION COVARIANCE
      //  =========================================================
      for (int i{0}; i < 6; i++) {
        for (j = 0; j < 10; j++) {
          y_tmp[j + 10 * i] = H[i + 6 * j];
        }
      }
      //  =========================================================
      //  9. KALMAN GAIN
      //  =========================================================
      for (int i{0}; i < 10; i++) {
        for (j = 0; j < 6; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den += state->P[i + 10 * R_measurement_tmp] *
                       y_tmp[R_measurement_tmp + 10 * j];
          }
          K[i + 10 * j] = yaw_den;
        }
      }
      for (int i{0}; i < 6; i++) {
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den += H[i + 6 * R_measurement_tmp] *
                       state->P[R_measurement_tmp + 10 * j];
          }
          b_H[i + 6 * j] = yaw_den;
        }
        for (j = 0; j < 6; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den += b_H[i + 6 * R_measurement_tmp] *
                       y_tmp[R_measurement_tmp + 10 * j];
          }
          R_position1_tmp = i + 6 * j;
          c_H[R_position1_tmp] = yaw_den + R_measurement[R_position1_tmp];
        }
      }
      coder::internal::mrdiv(K, c_H);
      //  =========================================================
      //  10. STATE CORRECTION
      //  =========================================================
      //  DEBUG
      // {
      // disp("P_bq:");
      // disp(P_prior(8:10,4:7));
      //
      // disp("K bias rows:");
      // disp(K(8:10,:));
      //
      // disp("innovation:");
      // disp(innovation);
      // }
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
      sinPhi = std::sin(GNSS->lla2[0]);
      t = 6.378137E+6 /
          std::sqrt(1.0 - 0.0066943799901413165 * sinPhi * sinPhi);
      //  =========================================================
      //  4. LLA -> ECEF
      //  ==========================================================
      //  =========================================================
      //  5. OUTPUT VECTOR
      //  ==========================================================
      psi_mid = (t + GNSS->lla2[2]) * std::cos(GNSS->lla2[0]);
      absxk = psi_mid * std::cos(GNSS->lla2[1]);
      omega_norm = psi_mid * std::sin(GNSS->lla2[1]);
      sinPhi *= t * 0.99330562000985867 + GNSS->lla2[2];
      for (int i{0}; i < 4; i++) {
        dv[i] = ((T_enu_ecef[i] * absxk + T_enu_ecef[i + 4] * omega_norm) +
                 T_enu_ecef[i + 8] * sinPhi) +
                T_enu_ecef[i + 12];
      }
      omega_norm = q_idx_3 * q_idx_3;
      theta = q_idx_2 * q_idx_2;
      H_q[0] = 1.0 - 2.0 * (theta + omega_norm);
      psi_mid = dt * q_idx_2;
      absxk = q_idx_0 * q_idx_3;
      H_q[3] = 2.0 * (psi_mid - absxk);
      t = dt * q_idx_3;
      yaw_den = q_idx_0 * q_idx_2;
      H_q[6] = 2.0 * (t + yaw_den);
      H_q[1] = 2.0 * (psi_mid + absxk);
      psi_mid = dt * dt;
      H_q[4] = 1.0 - 2.0 * (psi_mid + omega_norm);
      omega_norm = q_idx_2 * q_idx_3;
      absxk = q_idx_0 * dt;
      H_q[7] = 2.0 * (omega_norm - absxk);
      H_q[2] = 2.0 * (t - yaw_den);
      H_q[5] = 2.0 * (omega_norm + absxk);
      H_q[8] = 1.0 - 2.0 * (psi_mid + theta);
      yaw_den = GNSS->baseline_body[0];
      absxk = GNSS->baseline_body[1];
      t = GNSS->baseline_body[2];
      for (int i{0}; i < 3; i++) {
        psi_mid = delta_q[i];
        b_delta_q[i] = psi_mid - state->p[i];
        b_delta_q[i + 3] =
            (dv[i] - psi_mid) -
            ((H_q[i] * yaw_den + H_q[i + 3] * absxk) + H_q[i + 6] * t);
      }
      // {
      //
      // disp("delta_bg:");
      // disp(delta_x(8:10));
      // }
      //  =========================================================
      //  11. NORMALIZE QUATERNION
      //  =========================================================
      //  12. JOSEPH-FORM COVARIANCE UPDATE
      //  =========================================================
      for (int i{0}; i < 10; i++) {
        yaw_den = 0.0;
        for (j = 0; j < 6; j++) {
          yaw_den += K[i + 10 * j] * b_delta_q[j];
        }
        delta_x[i] = yaw_den;
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 6;
               R_measurement_tmp++) {
            yaw_den +=
                K[i + 10 * R_measurement_tmp] * H[R_measurement_tmp + 6 * j];
          }
          R_position1_tmp = i + 10 * j;
          F[R_position1_tmp] =
              static_cast<double>(b_I[R_position1_tmp]) - yaw_den;
        }
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den += F[i + 10 * R_measurement_tmp] *
                       state->P[R_measurement_tmp + 10 * j];
          }
          b_F[i + 10 * j] = yaw_den;
        }
        for (j = 0; j < 6; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 6;
               R_measurement_tmp++) {
            yaw_den += K[i + 10 * R_measurement_tmp] *
                       R_measurement[R_measurement_tmp + 6 * j];
          }
          y_tmp[i + 10 * j] = yaw_den;
        }
      }
      state->p[0] += delta_x[0];
      state->p[1] += delta_x[1];
      state->p[2] += delta_x[2];
      q_prior[0] += delta_x[3];
      q_prior[1] += delta_x[4];
      q_prior[2] += delta_x[5];
      q_prior[3] += delta_x[6];
      state->bg[0] += delta_x[7];
      state->bg[1] += delta_x[8];
      state->bg[2] += delta_x[9];
      yaw_den = coder::b_norm(q_prior);
      state->q[0] = q_prior[0] / yaw_den;
      state->q[1] = q_prior[1] / yaw_den;
      state->q[2] = q_prior[2] / yaw_den;
      state->q[3] = q_prior[3] / yaw_den;
      for (int i{0}; i < 10; i++) {
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 10;
               R_measurement_tmp++) {
            yaw_den +=
                b_F[i + 10 * R_measurement_tmp] * F[j + 10 * R_measurement_tmp];
          }
          P_posterior[i + 10 * j] = yaw_den;
        }
      }
      for (int i{0}; i < 10; i++) {
        for (j = 0; j < 10; j++) {
          yaw_den = 0.0;
          for (R_measurement_tmp = 0; R_measurement_tmp < 6;
               R_measurement_tmp++) {
            yaw_den += y_tmp[i + 10 * R_measurement_tmp] *
                       K[j + 10 * R_measurement_tmp];
          }
          F[i + 10 * j] = yaw_den;
        }
      }
      for (int i{0}; i < 100; i++) {
        P_posterior[i] += F[i];
      }
      //  Optional numerical symmetry enforcement
      for (int i{0}; i < 10; i++) {
        for (j = 0; j < 10; j++) {
          R_measurement_tmp = j + 10 * i;
          state->P[R_measurement_tmp] =
              0.5 * (P_posterior[R_measurement_tmp] + P_posterior[i + 10 * j]);
        }
      }
      //  DEBUG
      // {
      // fprintf("||P_bq|| = %.12e\n", ...
      //     norm(P_prior(8:10,4:7), 'fro'));
      //
      // fprintf("||K_bg|| = %.12e\n", ...
      //     norm(K(8:10,:), 'fro'));
      //
      // fprintf("||innovation|| = %.12e\n", ...
      //     norm(innovation));
      //
      // fprintf("delta_bg = [%+.12e %+.12e %+.12e]\n", ...
      //     delta_x(8), delta_x(9), delta_x(10));
      //
      // }
      //  =========================================================
      //  13. STORE POSTERIOR STATE
      //  =========================================================
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
  prev_velocity = 0.0;
}

// End of code generation (EKF_func.cpp)
