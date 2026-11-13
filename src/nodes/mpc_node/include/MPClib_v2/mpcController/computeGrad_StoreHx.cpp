//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGrad_StoreHx.cpp
//
// Code generation for function 'computeGrad_StoreHx'
//

// Include files
#include "computeGrad_StoreHx.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "xgemv.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace Objective {
void computeGrad_StoreHx(struct_T &obj, const double H[400], const double f[21],
                         const double x[21])
{
  switch (obj.objtype) {
  case 5: {
    int i;
    i = obj.nvar;
    if (i - 2 >= 0) {
      std::memset(&obj.grad[0], 0,
                  static_cast<unsigned int>(i - 1) * sizeof(double));
    }
    obj.grad[obj.nvar - 1] = obj.gammaScalar;
  } break;
  case 3: {
    int i;
    internal::blas::xgemv(obj.nvar, obj.nvar, H, obj.nvar, x, obj.Hx);
    i = static_cast<unsigned char>(obj.nvar);
    if (i - 1 >= 0) {
      std::copy(&obj.Hx[0], &obj.Hx[i], &obj.grad[0]);
    }
    if (obj.hasLinear && (obj.nvar >= 1)) {
      int ixlast;
      ixlast = obj.nvar - 1;
      for (int k{0}; k <= ixlast; k++) {
        obj.grad[k] += f[k];
      }
    }
  } break;
  default: {
    int i;
    int ixlast;
    internal::blas::xgemv(obj.nvar, obj.nvar, H, obj.nvar, x, obj.Hx);
    i = obj.nvar + 1;
    for (ixlast = i; ixlast < 21; ixlast++) {
      obj.Hx[ixlast - 1] = obj.beta * x[ixlast - 1];
    }
    std::copy(&obj.Hx[0], &obj.Hx[20], &obj.grad[0]);
    if (obj.hasLinear && (obj.nvar >= 1)) {
      ixlast = obj.nvar - 1;
      for (int k{0}; k <= ixlast; k++) {
        obj.grad[k] += f[k];
      }
    }
    if (20 - obj.nvar >= 1) {
      ixlast = obj.nvar;
      i = 19 - obj.nvar;
      for (int k{0}; k <= i; k++) {
        int i1;
        i1 = ixlast + k;
        obj.grad[i1] += obj.rho;
      }
    }
  } break;
  }
}

} // namespace Objective
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeGrad_StoreHx.cpp)
