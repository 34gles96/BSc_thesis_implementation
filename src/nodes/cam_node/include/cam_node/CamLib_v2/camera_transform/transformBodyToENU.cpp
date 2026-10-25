//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// transformBodyToENU.cpp
//
// Code generation for function 'transformBodyToENU'
//

// Include files
#include "transformBodyToENU.h"
#include <cmath>

// Function Definitions
void transformBodyToENU(const double p_body[3], const double p_cog_enu[3],
                        double q[4], double p_enu[3])
{
  double dv[9];
  double absxk;
  double d;
  double d1;
  double scale;
  double t;
  double y;
  //
  //  Transform a 3D point from Body/CoG frame into ENU.
  //
  //  Inputs:
  //  p_body    [3x1] 3D trajectory point in Body frame [m]
  //  p_cog_enu [3x1] EKF-estimated CoG position in ENU [m]
  //  q         [4x1] Body -> ENU quaternion
  //                   [qw; qx; qy; qz]
  //
  //  Output:
  //  p_enu     [3x1] 3D trajectory point in ENU [m]
  //
  //  Body frame:
  //    X = forward, Y = left, Z = up
  //
  //  ENU frame:
  //    X = East, Y = North, Z = Up
  //  Normalize quaternion
  scale = 3.3121686421112381E-170;
  absxk = std::abs(q[0]);
  if (absxk > 3.3121686421112381E-170) {
    y = 1.0;
    scale = absxk;
  } else {
    t = absxk / 3.3121686421112381E-170;
    y = t * t;
  }
  absxk = std::abs(q[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(q[2]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  absxk = std::abs(q[3]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }
  y = scale * std::sqrt(y);
  q[0] /= y;
  q[1] /= y;
  q[2] /= y;
  q[3] /= y;
  //  Body -> ENU rotation matrix
  //  Rotate body point into ENU and add CoG translation
  scale = q[3] * q[3];
  absxk = q[2] * q[2];
  dv[0] = 1.0 - 2.0 * (absxk + scale);
  t = q[1] * q[2];
  y = q[0] * q[3];
  dv[3] = 2.0 * (t - y);
  d = q[1] * q[3];
  d1 = q[0] * q[2];
  dv[6] = 2.0 * (d + d1);
  dv[1] = 2.0 * (t + y);
  t = q[1] * q[1];
  dv[4] = 1.0 - 2.0 * (t + scale);
  scale = q[2] * q[3];
  y = q[0] * q[1];
  dv[7] = 2.0 * (scale - y);
  dv[2] = 2.0 * (d - d1);
  dv[5] = 2.0 * (scale + y);
  dv[8] = 1.0 - 2.0 * (t + absxk);
  scale = p_body[0];
  absxk = p_body[1];
  t = p_body[2];
  for (int i{0}; i < 3; i++) {
    p_enu[i] =
        ((dv[i] * scale + dv[i + 3] * absxk) + dv[i + 6] * t) + p_cog_enu[i];
  }
}

// End of code generation (transformBodyToENU.cpp)
