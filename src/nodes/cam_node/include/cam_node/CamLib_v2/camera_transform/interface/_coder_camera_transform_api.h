//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_camera_transform_api.h
//
// Code generation for function 'initializeCameraToCoGTransform'
//

#ifndef _CODER_CAMERA_TRANSFORM_API_H
#define _CODER_CAMERA_TRANSFORM_API_H

// Include files
#include "emlrt.h"
#include "mex.h"
#include "tmwtypes.h"
#include <algorithm>
#include <cstring>

// Variable Declarations
extern emlrtCTX emlrtRootTLSGlobal;
extern emlrtContext emlrtContextGlobal;

// Function Declarations
void camera_transform_atexit();

void camera_transform_initialize();

void camera_transform_terminate();

void camera_transform_xil_shutdown();

void camera_transform_xil_terminate();

void initializeCameraToCoGTransform(real_T camera_pos_V[3],
                                    real_T camera_dir_V[3],
                                    real_T camera_up_V[3], real_T cog_pos_V[3],
                                    real_T T_body_camera[16]);

void initializeCameraToCoGTransform_api(const mxArray *const prhs[4],
                                        const mxArray **plhs);

void transformBodyToENU(real_T p_body[3], real_T p_cog_enu[3], real_T q[4],
                        real_T p_enu[3]);

void transformBodyToENU_api(const mxArray *const prhs[3], const mxArray **plhs);

void transformCameraToCoG(real_T p_camera[3], real_T T_body_camera[16],
                          real_T p_body[3]);

void transformCameraToCoG_api(const mxArray *const prhs[2],
                              const mxArray **plhs);

#endif
// End of code generation (_coder_camera_transform_api.h)
