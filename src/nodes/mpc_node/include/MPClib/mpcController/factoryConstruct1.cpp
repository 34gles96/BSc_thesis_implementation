//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct1.cpp
//
// Code generation for function 'factoryConstruct1'
//

// Include files
#include "factoryConstruct1.h"
#include "anonymous_function.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun, int nVar,
                      const array<double, 1U> &lb, const array<double, 1U> &ub,
                      k_struct_T &obj)
{
  int idx;
  boolean_T b;
  obj.objfun = objfun;
  obj.f_1 = 0.0;
  obj.f_2 = 0.0;
  obj.nVar = nVar;
  obj.mIneq = 0;
  obj.mEq = 0;
  obj.numEvals = 0;
  obj.SpecifyObjectiveGradient = false;
  obj.SpecifyConstraintGradient = false;
  obj.isEmptyNonlcon = true;
  obj.hasLB.set_size(nVar);
  obj.hasUB.set_size(nVar);
  obj.FiniteDifferenceType = 0;
  b = false;
  idx = 0;
  switch (static_cast<unsigned int>(ub.size(0) == 0) << 1 |
          static_cast<unsigned int>(lb.size(0) == 0)) {
  case 0U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
      obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
      if (obj.hasLB[idx] || obj.hasUB[idx]) {
        b = true;
      }
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
      obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
      idx++;
    }
    break;
  case 1U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB[idx] = false;
      obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
      b = obj.hasUB[idx];
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB[idx] = false;
      obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
      idx++;
    }
    break;
  case 2U:
    while ((!b) && (idx + 1 <= nVar)) {
      obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
      obj.hasUB[idx] = false;
      b = obj.hasLB[idx];
      idx++;
    }
    while (idx + 1 <= nVar) {
      obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
      obj.hasUB[idx] = false;
      idx++;
    }
    break;
  default:
    for (idx = 0; idx < nVar; idx++) {
      obj.hasLB[idx] = false;
      obj.hasUB[idx] = false;
    }
    break;
  }
  obj.hasBounds = b;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factoryConstruct1.cpp)
