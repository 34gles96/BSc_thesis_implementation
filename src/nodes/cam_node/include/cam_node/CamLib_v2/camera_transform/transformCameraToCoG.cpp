//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// transformCameraToCoG.cpp
//
// Code generation for function 'transformCameraToCoG'
//

// Include files
#include "transformCameraToCoG.h"

// Function Definitions
void transformCameraToCoG(const double p_camera[3],
                          const double T_body_camera[16], double p_body[3])
{
  double b_T_body_camera[4];
  double p_camera_idx_0;
  double p_camera_idx_1;
  double p_camera_idx_2;
  //
  //  Transform a 3D point from Camera to Body/CoG frame.
  //
  //  Inputs:
  //  p_camera      [3x1] Point in camera optical frame [m]
  //  T_body_camera [4x4] Camera -> Body transformation
  //
  //  Output:
  //  p_body        [3x1] Point in Body/CoG frame [m]
  //  Homogeneous camera point
  //  Apply homogeneous transformation
  //  Extract Cartesian coordinates
  p_camera_idx_0 = p_camera[0];
  p_camera_idx_1 = p_camera[1];
  p_camera_idx_2 = p_camera[2];
  for (int i{0}; i < 4; i++) {
    b_T_body_camera[i] = ((T_body_camera[i] * p_camera_idx_0 +
                           T_body_camera[i + 4] * p_camera_idx_1) +
                          T_body_camera[i + 8] * p_camera_idx_2) +
                         T_body_camera[i + 12];
  }
  p_body[0] = b_T_body_camera[0];
  p_body[1] = b_T_body_camera[1];
  p_body[2] = b_T_body_camera[2];
}

// End of code generation (transformCameraToCoG.cpp)
