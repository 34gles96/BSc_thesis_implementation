//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct.cpp
//
// Code generation for function 'factoryConstruct'
//

// Include files
#include "factoryConstruct.h"
#include "anonymous_function.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace FiniteDifferences {
void factoryConstruct(const anonymous_function &objfun, const double lb[20],
                      const double ub[20], k_struct_T &obj)
{
  int idx;
  boolean_T b;
  obj.objfun = objfun;
  obj.f_1 = 0.0;
  obj.numEvals = 0;
  b = false;
  idx = 0;
  while ((!b) && (idx + 1 <= 20)) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    if (obj.hasLB[idx] || obj.hasUB[idx]) {
      b = true;
    }
    idx++;
  }
  while (idx + 1 <= 20) {
    obj.hasLB[idx] = ((!std::isinf(lb[idx])) && (!std::isnan(lb[idx])));
    obj.hasUB[idx] = ((!std::isinf(ub[idx])) && (!std::isnan(ub[idx])));
    idx++;
  }
  obj.hasBounds = b;
}

} // namespace FiniteDifferences
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (factoryConstruct.cpp)
