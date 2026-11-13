//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_mpcController_api.h
//
// Code generation for function 'mpcController'
//

#ifndef _CODER_MPCCONTROLLER_API_H
#define _CODER_MPCCONTROLLER_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Type Definitions
struct struct0_T {
  real_T Np;
  real_T Ts;
  real_T L;
  real_T Q[16];
  real_T R[4];
  real_T S[4];
  real_T deltaMax;
  real_T aMin;
  real_T aMax;
};

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void mpcController(real_T x_current[4], real_T refHorizon[40],
                   real_T u_last_applied[2], struct0_T *params,
                   real_T u_optimal[2], real_T U_optimal[20]);

void mpcController_api(const mxArray *const prhs[4], int32_T nlhs,
                       const mxArray *plhs[2]);

void mpcController_atexit();

void mpcController_initialize();

void mpcController_terminate();

void mpcController_xil_shutdown();

void mpcController_xil_terminate();

#endif
// End of code generation (_coder_mpcController_api.h)
