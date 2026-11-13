//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController_types.h
//
// Code generation for function 'mpcController'
//

#ifndef MPCCONTROLLER_TYPES_H
#define MPCCONTROLLER_TYPES_H

// Include files
#include "rtwtypes.h"

// Type Definitions
struct struct0_T {
  double Np;
  double Ts;
  double L;
  double Q[16];
  double R[4];
  double S[4];
  double deltaMax;
  double aMin;
  double aMax;
};

#endif
// End of code generation (mpcController_types.h)
