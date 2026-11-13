//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fullColLDL2_.cpp
//
// Code generation for function 'fullColLDL2_'
//

// Include files
#include "fullColLDL2_.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void fullColLDL2_(e_struct_T &obj, int NColsRemain)
{
  for (int k{0}; k < NColsRemain; k++) {
    double alpha1;
    double y;
    int LD_diagOffset;
    int i;
    int jA;
    int offset1;
    int subMatrixDim;
    LD_diagOffset = 42 * k;
    alpha1 = -1.0 / obj.FMat[LD_diagOffset];
    subMatrixDim = (NColsRemain - k) - 2;
    offset1 = LD_diagOffset + 2;
    y = obj.workspace_;
    for (jA = 0; jA <= subMatrixDim; jA++) {
      y = obj.FMat[(LD_diagOffset + jA) + 1];
    }
    obj.workspace_ = y;
    if (!(alpha1 == 0.0)) {
      jA = LD_diagOffset;
      for (int j{0}; j <= subMatrixDim; j++) {
        if (y != 0.0) {
          double temp;
          int i1;
          temp = y * alpha1;
          i = jA + 43;
          i1 = subMatrixDim + jA;
          for (int ijA{i}; ijA <= i1 + 43; ijA++) {
            obj.FMat[ijA - 1] += y * temp;
          }
        }
        jA += 41;
      }
    }
    alpha1 = 1.0 / obj.FMat[LD_diagOffset];
    i = LD_diagOffset + subMatrixDim;
    for (jA = offset1; jA <= i + 2; jA++) {
      obj.FMat[jA - 1] *= alpha1;
    }
  }
}

} // namespace DynamicRegCholManager
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (fullColLDL2_.cpp)
