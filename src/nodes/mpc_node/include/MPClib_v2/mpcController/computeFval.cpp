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
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
double computeFval(const struct_T &obj, double workspace[861],
                   const double H[400], const double f[21], const double x[21])
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
      int i;
      i = static_cast<unsigned char>(obj.nvar);
      for (int idx{0}; idx < i; idx++) {
        val += x[idx] * workspace[idx];
      }
    }
  } break;
  default: {
    int i;
    linearForm_(obj.hasLinear, obj.nvar, workspace, H, f, x);
    i = obj.nvar + 1;
    for (int idx{i}; idx < 21; idx++) {
      workspace[idx - 1] = 0.5 * obj.beta * x[idx - 1] + obj.rho;
    }
    val = 0.0;
    for (int idx{0}; idx < 20; idx++) {
      val += x[idx] * workspace[idx];
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
