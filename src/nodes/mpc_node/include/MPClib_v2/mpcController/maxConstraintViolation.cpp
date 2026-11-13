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
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
double maxConstraintViolation(const h_struct_T &obj, const double x[861],
                              int ix0)
{
  double v;
  int i;
  int idxLB;
  v = 0.0;
  if (obj.sizes[3] > 0) {
    i = static_cast<unsigned char>(obj.sizes[3]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexLB[idx] - 1;
      v = std::fmax(v, -x[(ix0 + idxLB) - 1] - obj.lb[idxLB]);
    }
  }
  if (obj.sizes[4] > 0) {
    i = static_cast<unsigned char>(obj.sizes[4]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexUB[idx] - 1;
      v = std::fmax(v, x[(ix0 + idxLB) - 1] - obj.ub[idxLB]);
    }
  }
  if (obj.sizes[0] > 0) {
    i = static_cast<unsigned char>(obj.sizes[0]);
    for (int idx{0}; idx < i; idx++) {
      v = std::fmax(v, std::abs(x[(ix0 + obj.indexFixed[idx]) - 2] -
                                obj.ub[obj.indexFixed[idx] - 1]));
    }
  }
  return v;
}

double maxConstraintViolation(const h_struct_T &obj, const double x[21])
{
  double v;
  int i;
  int idxLB;
  v = 0.0;
  if (obj.sizes[3] > 0) {
    i = static_cast<unsigned char>(obj.sizes[3]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexLB[idx] - 1;
      v = std::fmax(v, -x[idxLB] - obj.lb[idxLB]);
    }
  }
  if (obj.sizes[4] > 0) {
    i = static_cast<unsigned char>(obj.sizes[4]);
    for (int idx{0}; idx < i; idx++) {
      idxLB = obj.indexUB[idx] - 1;
      v = std::fmax(v, x[idxLB] - obj.ub[idxLB]);
    }
  }
  if (obj.sizes[0] > 0) {
    i = static_cast<unsigned char>(obj.sizes[0]);
    for (int idx{0}; idx < i; idx++) {
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
