//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_func_types.h
//
// Code generation for function 'EKF_func'
//

#ifndef EKF_FUNC_TYPES_H
#define EKF_FUNC_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct1_T {
  double x_sensor_body[3];
  double y_sensor_body[3];
  double z_sensor_body[3];
};

struct struct2_T {
  boolean_T valid;
  double timestamp;
  double lla1[3];
  double lla2[3];
  double position1_enu[3];
  double position2_enu[3];
  double position_enu[3];
  double baseline_enu[3];
  double baseline_body[3];
  double hAcc1;
  double vAcc1;
  double hAcc2;
  double vAcc2;
};

struct struct3_T {
  double velocity;
  double steering_angle;
  double wheelbase;
};

struct struct0_T {
  boolean_T valid;
  double timestamp;
  double gyro[3];
  double gyro_covariance[9];
  double accel[3];
  double accel_covariance[9];
  struct1_T mounting;
};

struct struct4_T {
  double p[3];
  double q[4];
  double bg[3];
  double P[100];
  double Q[100];
  double p_ecef0[3];
};

#endif
// End of code generation (EKF_func_types.h)
