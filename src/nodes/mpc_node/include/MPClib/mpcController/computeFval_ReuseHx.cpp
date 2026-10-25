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
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval_ReuseHx(const f_struct_T &obj, array<double, 2U> &workspace,
                           const array<double, 1U> &f,
                           const array<double, 1U> &x)
{
  double val;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x[obj.nvar - 1];
    break;
  case 3: {
    if (obj.hasLinear) {
      int ixlast;
      ixlast = obj.nvar;
      for (int k{0}; k < ixlast; k++) {
        workspace[k] = 0.5 * obj.Hx[k] + f[k];
      }
      val = 0.0;
      if (obj.nvar >= 1) {
        for (int k{0}; k < ixlast; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      val = 0.0;
      if (obj.nvar >= 1) {
        int ixlast;
        ixlast = obj.nvar;
        for (int k{0}; k < ixlast; k++) {
          val += x[k] * obj.Hx[k];
        }
      }
      val *= 0.5;
    }
  } break;
  default: {
    int maxRegVar_tmp;
    maxRegVar_tmp = obj.maxVar - 1;
    if (obj.hasLinear) {
      int ixlast;
      ixlast = obj.nvar;
      for (int k{0}; k < ixlast; k++) {
        workspace[k] = f[k];
      }
      ixlast = obj.maxVar - obj.nvar;
      for (int k{0}; k <= ixlast - 2; k++) {
        workspace[obj.nvar + k] = obj.rho;
      }
      for (int k{0}; k < maxRegVar_tmp; k++) {
        workspace[k] = workspace[k] + 0.5 * obj.Hx[k];
      }
      val = 0.0;
      if (maxRegVar_tmp >= 1) {
        ixlast = obj.maxVar;
        for (int k{0}; k <= ixlast - 2; k++) {
          val += x[k] * workspace[k];
        }
      }
    } else {
      int ixlast;
      val = 0.0;
      if (maxRegVar_tmp >= 1) {
        ixlast = obj.maxVar;
        for (int k{0}; k <= ixlast - 2; k++) {
          val += x[k] * obj.Hx[k];
        }
      }
      val *= 0.5;
      ixlast = obj.nvar + 1;
      for (int k{ixlast}; k <= maxRegVar_tmp; k++) {
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
