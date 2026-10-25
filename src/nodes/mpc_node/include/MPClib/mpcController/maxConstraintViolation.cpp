//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// maxConstraintViolation.cpp
//
// Code generation for function 'maxConstraintViolation'
//

// Include files
#include "maxConstraintViolation.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(const i_struct_T &obj, const array<double, 1U> &x)
{
  double v;
  int mFixed;
  int mLB;
  int mUB;
  mLB = obj.sizes[3];
  mUB = obj.sizes[4];
  mFixed = obj.sizes[0];
  v = 0.0;
  if (obj.sizes[3] > 0) {
    for (int idx{0}; idx < mLB; idx++) {
      int idxLB;
      idxLB = obj.indexLB[idx] - 1;
      v = std::fmax(v, -x[idxLB] - obj.lb[idxLB]);
    }
  }
  if (obj.sizes[4] > 0) {
    for (int idx{0}; idx < mUB; idx++) {
      mLB = obj.indexUB[idx] - 1;
      v = std::fmax(v, x[mLB] - obj.ub[mLB]);
    }
  }
  if (obj.sizes[0] > 0) {
    for (int idx{0}; idx < mFixed; idx++) {
      v = std::fmax(v, std::abs(x[obj.indexFixed[idx] - 1] -
                                obj.ub[obj.indexFixed[idx] - 1]));
    }
  }
  return v;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (maxConstraintViolation.cpp)
