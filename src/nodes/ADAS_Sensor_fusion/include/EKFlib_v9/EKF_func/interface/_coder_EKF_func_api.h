//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_EKF_func_api.h
//
// Code generation for function 'EKF_func'
//

#ifndef _CODER_EKF_FUNC_API_H
#define _CODER_EKF_FUNC_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct1_T {
  real_T x_sensor_body[3];
  real_T y_sensor_body[3];
  real_T z_sensor_body[3];
};

struct struct2_T {
  boolean_T valid;
  real_T timestamp;
  real_T lla1[3];
  real_T lla2[3];
  real_T position1_enu[3];
  real_T position2_enu[3];
  real_T position_enu[3];
  real_T baseline_enu[3];
  real_T baseline_body[3];
  real_T hAcc1;
  real_T vAcc1;
  real_T hAcc2;
  real_T vAcc2;
};

struct struct3_T {
  real_T velocity;
  real_T steering_angle;
  real_T wheelbase;
  real_T r_p1_body[3];
};

struct struct0_T {
  boolean_T valid;
  real_T timestamp;
  real_T gyro[3];
  real_T gyro_covariance[9];
  real_T accel[3];
  real_T accel_covariance[9];
  struct1_T mounting;
};

struct struct4_T {
  real_T p[3];
  real_T q[4];
  real_T bg[3];
  real_T P[100];
  real_T Q[100];
  real_T p_ecef0[3];
  real_T PosENU_cog[3];
  real_T heading;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void EKF_func(struct0_T *IMU, struct2_T *GNSS, struct3_T *Vehicle,
              struct4_T *state);

void EKF_func_api(const mxArray *const prhs[4], const mxArray **plhs);

void EKF_func_atexit();

void EKF_func_initialize();

void EKF_func_terminate();

void EKF_func_xil_shutdown();

void EKF_func_xil_terminate();

#endif
// End of code generation (_coder_EKF_func_api.h)
