//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// initializeCameraToCoGTransform.h
//
// Code generation for function 'initializeCameraToCoGTransform'
//

#ifndef INITIALIZECAMERATOCOGTRANSFORM_H
#define INITIALIZECAMERATOCOGTRANSFORM_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void initializeCameraToCoGTransform(const double camera_pos_V[3],
                                           const double camera_dir_V[3],
                                           const double camera_up_V[3],
                                           const double cog_pos_V[3],
                                           double T_body_camera[16]);

#endif
// End of code generation (initializeCameraToCoGTransform.h)
