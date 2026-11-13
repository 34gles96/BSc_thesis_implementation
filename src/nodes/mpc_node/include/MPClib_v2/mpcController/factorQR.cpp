//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factorQR.cpp
//
// Code generation for function 'factorQR'
//

// Include files
#include "factorQR.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace QRManager {
void factorQR(d_struct_T &obj, const double A[861], int mrows, int ncols)
{
  int i;
  boolean_T guard1;
  i = mrows * ncols;
  guard1 = false;
  if (i > 0) {
    for (int idx{0}; idx < ncols; idx++) {
      int ix0;
      int iy0;
      ix0 = 21 * idx;
      iy0 = 41 * idx;
      i = static_cast<unsigned char>(mrows);
      for (int k{0}; k < i; k++) {
        obj.QR[iy0 + k] = A[ix0 + k];
      }
    }
    guard1 = true;
  } else if (i == 0) {
    obj.mrows = mrows;
    obj.ncols = ncols;
    obj.minRowCol = 0;
  } else {
    guard1 = true;
  }
  if (guard1) {
    obj.usedPivoting = false;
    obj.mrows = mrows;
    obj.ncols = ncols;
    for (int idx{0}; idx < ncols; idx++) {
      obj.jpvt[idx] = idx + 1;
    }
    if (mrows <= ncols) {
      i = mrows;
    } else {
      i = ncols;
    }
    obj.minRowCol = i;
    std::memset(&obj.tau[0], 0, 41U * sizeof(double));
    if (i >= 1) {
      internal::reflapack::qrf(obj.QR, mrows, ncols, i, obj.tau);
    }
  }
}

} // namespace QRManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factorQR.cpp)
