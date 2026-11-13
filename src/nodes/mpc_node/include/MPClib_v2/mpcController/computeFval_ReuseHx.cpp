//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval_ReuseHx.cpp
//
// Code generation for function 'computeFval_ReuseHx'
//

// Include files
#include "computeFval_ReuseHx.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const struct_T &obj, double workspace[861],
                           const double f[21], const double x[21])
{
  double val;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x[obj.nvar - 1];
    break;
  case 3: {
    if (obj.hasLinear) {
      int i;
      i = static_cast<unsigned char>(obj.nvar);
      for (int k{0}; k < i; k++) {
        workspace[k] = 0.5 * obj.Hx[k] + f[k];
      }
      val = 0.0;
      if (obj.nvar >= 1) {
        for (int k{0}; k < i; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj.nvar >= 1) {
        int i;
        i = static_cast<unsigned char>(obj.nvar);
        for (int k{0}; k < i; k++) {
          val += x[k] * obj.Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    if (obj.hasLinear) {
      int i;
      i = static_cast<unsigned char>(obj.nvar);
      if (i - 1 >= 0) {
        std::copy(&f[0], &f[i], &workspace[0]);
      }
      i = 19 - obj.nvar;
      for (int k{0}; k <= i; k++) {
        workspace[obj.nvar + k] = obj.rho;
      }
      val = 0.0;
      for (int k{0}; k < 20; k++) {
        double d;
        d = workspace[k] + 0.5 * obj.Hx[k];
        workspace[k] = d;
        val += x[k] * d;
      }
    } else {
      int i;
      val = 0.0;
      for (int k{0}; k < 20; k++) {
        val += x[k] * obj.Hx[k];
      }
      val *= 0.5;
      i = obj.nvar + 1;
      for (int k{i}; k < 21; k++) {
        val += x[k - 1] * obj.rho;
      }
    }
  } break;
  }
  return val;
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeFval_ReuseHx.cpp)
