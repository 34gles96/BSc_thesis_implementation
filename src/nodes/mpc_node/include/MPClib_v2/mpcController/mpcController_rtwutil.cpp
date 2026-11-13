//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController_rtwutil.cpp
//
// Code generation for function 'mpcController_rtwutil'
//

// Include files
#include "mpcController_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
int div_nde_s32_floor(int numerator, int denominator)
{
  int i;
  if (((numerator < 0) != (denominator < 0)) &&
      (numerator % denominator != 0)) {
    i = -1;
  } else {
    i = 0;
  }
  return numerator / denominator + i;
}

// End of code generation (mpcController_rtwutil.cpp)
