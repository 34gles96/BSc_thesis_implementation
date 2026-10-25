//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_camera_transform_mex.h
//
// Code generation for function 'initializeCameraToCoGTransform'
//

#ifndef _CODER_CAMERA_TRANSFORM_MEX_H
#define _CODER_CAMERA_TRANSFORM_MEX_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"

// Function Declarations
MEXFUNCTION_LINKAGE void mexFunction(int32_T nlhs, mxArray *plhs[],
                                     int32_T nrhs, const mxArray *prhs[]);

emlrtCTX mexFunctionCreateRootTLS();

void unsafe_initializeCameraToCoGTransform_mexFunction(int32_T nlhs,
                                                       mxArray *plhs[1],
                                                       int32_T nrhs,
                                                       const mxArray *prhs[4]);

void unsafe_transformBodyToENU_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                           int32_T nrhs,
                                           const mxArray *prhs[3]);

void unsafe_transformCameraToCoG_mexFunction(int32_T nlhs, mxArray *plhs[1],
                                             int32_T nrhs,
                                             const mxArray *prhs[2]);

#endif
// End of code generation (_coder_camera_transform_mex.h)
