//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mrdivide_helper.h
//
// Code generation for function 'mrdivide_helper'
//

#ifndef MRDIVIDE_HELPER_H
#define MRDIVIDE_HELPER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace internal {
void mrdiv(const double A[12], const double B[9], double Y[12]);

void mrdiv(double A[60], const double B[36]);

} // namespace internal
} // namespace coder

#endif
// End of code generation (mrdivide_helper.h)
