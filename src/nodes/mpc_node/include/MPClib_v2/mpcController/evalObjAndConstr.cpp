//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// evalObjAndConstr.cpp
//
// Code generation for function 'evalObjAndConstr'
//

// Include files
#include "evalObjAndConstr.h"
#include "anonymous_function.h"
#include "mpcController.h"
#include "mpcController_internal_types1.h"
#include "rt_nonfinite.h"
#include "stickyStruct.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace utils {
namespace ObjNonlinEvaluator {
double evalObjAndConstr(const ::coder::internal::i_stickyStruct &obj,
                        const double x[20], int &status)
{
  double fval;
  boolean_T b;
  fval = mpcController_anonFcn1(
      obj.next.next.next.next.next.next.next.next.value.workspace.x_current,
      obj.next.next.next.next.next.next.next.next.value.workspace.refHorizon,
      obj.next.next.next.next.next.next.next.next.value.workspace
          .u_last_applied,
      obj.next.next.next.next.next.next.next.next.value.workspace.Ts,
      obj.next.next.next.next.next.next.next.next.value.workspace.L,
      obj.next.next.next.next.next.next.next.next.value.workspace.Q,
      obj.next.next.next.next.next.next.next.next.value.workspace.R,
      obj.next.next.next.next.next.next.next.next.value.workspace.S, x);
  status = 1;
  b = std::isnan(fval);
  if (std::isinf(fval) || b) {
    if (b) {
      status = -3;
    } else if (fval < 0.0) {
      status = -1;
    } else {
      status = -2;
    }
  }
  if (status == 1) {
    status = 1;
  }
  return fval;
}

} // namespace ObjNonlinEvaluator
} // namespace utils
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (evalObjAndConstr.cpp)
