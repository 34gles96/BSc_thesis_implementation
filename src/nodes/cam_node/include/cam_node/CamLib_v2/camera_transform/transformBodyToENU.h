//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// transformBodyToENU.h
//
// Code generation for function 'transformBodyToENU'
//

#ifndef TRANSFORMBODYTOENU_H
#define TRANSFORMBODYTOENU_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void transformBodyToENU(const double p_body[3],
                               const double p_cog_enu[3], double q[4],
                               double p_enu[3]);

#endif
// End of code generation (transformBodyToENU.h)
