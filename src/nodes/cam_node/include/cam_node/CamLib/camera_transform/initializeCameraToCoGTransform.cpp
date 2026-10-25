//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// initializeCameraToCoGTransform.cpp
//
// Code generation for function 'initializeCameraToCoGTransform'
//

// Include files
#include "initializeCameraToCoGTransform.h"
#include <cmath>
#include <cstring>

// Function Definitions
void initializeCameraToCoGTransform(const double camera_pos_V[3],
                                    const double camera_dir_V[3],
                                    const double camera_up_V[3],
                                    const double cog_pos_V[3],
                                    double T_body_camera[16])
{
  static const signed char a[9]{0, 1, 0, -1, 0, 0, 0, 0, 1};
  double x_camera_V[9];
  double z_camera_V[3];
  double absxk;
  double scale;
  double t;
  double x_camera_V_idx_0;
  double x_camera_V_idx_1;
  double x_camera_V_idx_2;
  double y;
  //
  //  Initialize Camera -> Body/CoG homogeneous transformation.
  //
  //  Inputs:
  //  camera_pos_V [3x1] Camera mounting position in BeamNG
  //  camera_dir_V [3x1] Camera forward direction in BeamNG
  //  camera_up_V  [3x1] Camera upward direction in BeamNG
  //  cog_pos_V    [3x1] CoG position in BeamNG vehicle frame
  //
  //  Output:
  //  T_body_camera [4x4] Camera -> Body/CoG transformation
  //
  //  Conventions:
  //  BeamNG: X left, Y backward, Z up
  //  Body:   X forward, Y left, Z up
  //  Camera: X right, Y down, Z forward
  //  1. BeamNG -> Body rotation
  //  2. Construct camera axes in BeamNG frame
  //  Camera Z-axis: forward
  scale = 3.3121686421112381E-170;
  absxk = std::abs(camera_dir_V[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs(camera_dir_V[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(camera_dir_V[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * std::sqrt(y);
  z_camera_V[0] = camera_dir_V[0] / y;
  z_camera_V[1] = camera_dir_V[1] / y;
  z_camera_V[2] = camera_dir_V[2] / y;
  //  Camera X-axis: right
  x_camera_V_idx_0 =
      z_camera_V[1] * camera_up_V[2] - camera_up_V[1] * z_camera_V[2];
  x_camera_V_idx_1 =
      camera_up_V[0] * z_camera_V[2] - z_camera_V[0] * camera_up_V[2];
  x_camera_V_idx_2 =
      z_camera_V[0] * camera_up_V[1] - camera_up_V[0] * z_camera_V[1];
  scale = 3.3121686421112381E-170;
  absxk = std::abs(x_camera_V_idx_0);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs(x_camera_V_idx_1);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(x_camera_V_idx_2);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * std::sqrt(y);
  x_camera_V_idx_0 /= y;
  x_camera_V_idx_1 /= y;
  x_camera_V_idx_2 /= y;
  //  Camera Y-axis: down
  //  Camera -> BeamNG rotation
  //  3. Camera -> Body rotation
  //  4. Camera mounting vector relative to CoG
  //  CoG -> Camera, expressed in BeamNG coordinates
  //  Express mounting vector in Body coordinates
  //  5. Construct homogeneous transformation
  std::memset(&T_body_camera[0], 0, 16U * sizeof(double));
  T_body_camera[0] = 1.0;
  T_body_camera[5] = 1.0;
  T_body_camera[10] = 1.0;
  T_body_camera[15] = 1.0;
  x_camera_V[3] =
      z_camera_V[1] * x_camera_V_idx_2 - x_camera_V_idx_1 * z_camera_V[2];
  x_camera_V[4] =
      x_camera_V_idx_0 * z_camera_V[2] - z_camera_V[0] * x_camera_V_idx_2;
  x_camera_V[5] =
      z_camera_V[0] * x_camera_V_idx_1 - x_camera_V_idx_0 * z_camera_V[1];
  x_camera_V[0] = x_camera_V_idx_0;
  x_camera_V[6] = z_camera_V[0];
  x_camera_V[1] = x_camera_V_idx_1;
  x_camera_V[7] = z_camera_V[1];
  x_camera_V[2] = x_camera_V_idx_2;
  x_camera_V[8] = z_camera_V[2];
  for (int i{0}; i < 3; i++) {
    signed char i1;
    signed char i2;
    signed char i3;
    i1 = a[i];
    i2 = a[i + 3];
    i3 = a[i + 6];
    for (int i4{0}; i4 < 3; i4++) {
      T_body_camera[i + (i4 << 2)] =
          (static_cast<double>(i1) * x_camera_V[3 * i4] +
           static_cast<double>(i2) * x_camera_V[3 * i4 + 1]) +
          static_cast<double>(i3) * x_camera_V[3 * i4 + 2];
    }
    z_camera_V[i] = camera_pos_V[i] - cog_pos_V[i];
  }
  scale = z_camera_V[0];
  absxk = z_camera_V[1];
  t = z_camera_V[2];
  for (int i{0}; i < 3; i++) {
    T_body_camera[i + 12] = (static_cast<double>(a[i]) * scale +
                             static_cast<double>(a[i + 3]) * absxk) +
                            static_cast<double>(a[i + 6]) * t;
  }
}

// End of code generation (initializeCameraToCoGTransform.cpp)
