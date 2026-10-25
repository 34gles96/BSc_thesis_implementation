//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// EKF_func.h
//
// Code generation for function 'EKF_func'
//

#ifndef EKF_FUNC_H
#define EKF_FUNC_H

// Include files
#include "EKF_func_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void EKF_func(const struct0_T *IMU, struct2_T *GNSS,
                     const struct3_T *Vehicle, struct4_T *state);

void EKF_func_init();

#endif
// End of code generation (EKF_func.h)
