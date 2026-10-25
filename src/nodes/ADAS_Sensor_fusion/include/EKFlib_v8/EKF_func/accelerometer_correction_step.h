//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// accelerometer_correction_step.h
//
// Code generation for function 'accelerometer_correction_step'
//

#ifndef ACCELEROMETER_CORRECTION_STEP_H
#define ACCELEROMETER_CORRECTION_STEP_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct struct4_T;

// Function Declarations
boolean_T accelerometer_correction_step(const double IMU_accel[3],
                                        const double IMU_accel_covariance[9],
                                        double Vehicle_velocity,
                                        double Vehicle_steering_angle,
                                        double Vehicle_wheelbase,
                                        struct4_T *state,
                                        double b_prev_velocity, double dt);

#endif
// End of code generation (accelerometer_correction_step.h)
