//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// transformCameraToCoG.h
//
// Code generation for function 'transformCameraToCoG'
//

#ifndef TRANSFORMCAMERATOCOG_H
#define TRANSFORMCAMERATOCOG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void transformCameraToCoG(const double p_camera[3],
                                 const double T_body_camera[16],
                                 double p_body[3]);

#endif
// End of code generation (transformCameraToCoG.h)
