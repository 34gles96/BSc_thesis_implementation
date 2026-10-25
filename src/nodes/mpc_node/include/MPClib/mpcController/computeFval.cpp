//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeFval.cpp
//
// Code generation for function 'computeFval'
//

// Include files
#include "computeFval.h"
#include "linearForm_.h"
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
double computeFval(const f_struct_T &obj, array<double, 2U> &workspace,
                   const array<double, 2U> &H, const array<double, 1U> &f,
                   const array<double, 1U> &x)
{
  double val;
  switch (obj.objtype) {
  case 5:
    val = obj.gammaScalar * x[obj.nvar - 1];
    break;
  case 3: {
    linearForm_(obj.hasLinear, obj.nvar, workspace, H, f, x);
    val = 0.0;
    if (obj.nvar >= 1) {
      int ixlast;
      ixlast = obj.nvar;
      for (int k{0}; k < ixlast; k++) {
        val += x[k] * workspace[k];
      }
    }
  } break;
  default: {
    int ixlast;
    int k;
    linearForm_(obj.hasLinear, obj.nvar, workspace, H, f, x);
    ixlast = obj.nvar + 1;
    k = obj.maxVar - 1;
    for (int idx{ixlast}; idx <= k; idx++) {
      workspace[idx - 1] = 0.5 * obj.beta * x[idx - 1] + obj.rho;
    }
    val = 0.0;
    if (k >= 1) {
      ixlast = obj.maxVar;
      for (k = 0; k <= ixlast - 2; k++) {
        val += x[k] * workspace[k];
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

// End of code generation (computeFval.cpp)
