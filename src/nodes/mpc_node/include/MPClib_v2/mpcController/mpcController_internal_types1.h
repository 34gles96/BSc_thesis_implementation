//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController_internal_types1.h
//
// Code generation for function 'mpcController_internal_types1'
//

#ifndef MPCCONTROLLER_INTERNAL_TYPES1_H
#define MPCCONTROLLER_INTERNAL_TYPES1_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Definitions
struct f_struct_T {
  double x_current[4];
  double refHorizon[40];
  double u_last_applied[2];
  double Ts;
  double L;
  double Q[16];
  double R[4];
  double S[4];
};

#endif
// End of code generation (mpcController_internal_types1.h)
