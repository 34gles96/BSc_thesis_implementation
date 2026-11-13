//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// setProblemType.cpp
//
// Code generation for function 'setProblemType'
//

// Include files
#include "setProblemType.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(h_struct_T &obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj.nVar = 20;
    obj.mConstr = obj.mConstrOrig;
    if (obj.nWConstr[4] > 0) {
      i = static_cast<unsigned char>(obj.sizesNormal[4]);
      for (int idx{0}; idx < i; idx++) {
        obj.isActiveConstr[(obj.isActiveIdxNormal[4] + idx) - 1] =
            obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1];
      }
    }
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesNormal[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxNormal[i];
    }
  } break;
  case 1: {
    int i;
    int idxStartIneq;
    obj.nVar = 21;
    obj.mConstr = obj.mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxPhaseOne[i];
    }
    i = static_cast<unsigned char>(obj.sizes[0]);
    for (int idx{0}; idx < i; idx++) {
      obj.ATwset[21 * idx + 20] = 0.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = 21;
    obj.lb[20] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    i = obj.nActiveConstr;
    for (int idx{idxStartIneq}; idx <= i; idx++) {
      obj.ATwset[21 * (idx - 1) + 20] = -1.0;
    }
    if (obj.nWConstr[4] > 0) {
      i = static_cast<unsigned char>(obj.sizesNormal[4] + 1);
      for (int idx{0}; idx < i; idx++) {
        obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj.isActiveConstr[obj.isActiveIdx[4] - 2] = false;
  } break;
  case 2: {
    int i;
    obj.nVar = 20;
    obj.mConstr = 40;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegularized[i];
    }
    if (obj.probType != 4) {
      int i1;
      int idx;
      int idxStartIneq;
      int idx_lb;
      idx_lb = 20;
      i = obj.sizesNormal[3] + 1;
      i1 = obj.sizesRegularized[3];
      for (idx = i; idx <= i1; idx++) {
        idx_lb++;
        obj.indexLB[idx - 1] = idx_lb;
      }
      if (obj.nWConstr[4] > 0) {
        i = static_cast<unsigned char>(obj.sizesRegularized[4]);
        for (idx = 0; idx < i; idx++) {
          obj.isActiveConstr[obj.isActiveIdxRegularized[4] + idx] =
              obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1];
        }
      }
      i = obj.isActiveIdx[4];
      i1 = obj.isActiveIdxRegularized[4] - 1;
      if (i <= i1) {
        std::memset(&obj.isActiveConstr[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) *
                        sizeof(boolean_T));
      }
      idxStartIneq = obj.isActiveIdx[2];
      i = obj.nActiveConstr;
      for (int idx_col{idxStartIneq}; idx_col <= i; idx_col++) {
        idx_lb = 21 * (idx_col - 1) - 1;
        if (obj.Wid[idx_col - 1] == 3) {
          i1 = obj.Wlocalidx[idx_col - 1];
          idx = i1 + 19;
          if (idx >= 21) {
            std::memset(
                &obj.ATwset[idx_lb + 21], 0,
                static_cast<unsigned int>(((idx + idx_lb) - idx_lb) - 20) *
                    sizeof(double));
          }
          obj.ATwset[(i1 + idx_lb) + 20] = -1.0;
          i1 += 21;
          if (i1 <= 20) {
            std::memset(
                &obj.ATwset[i1 + idx_lb], 0,
                static_cast<unsigned int>(((idx_lb - i1) - idx_lb) + 21) *
                    sizeof(double));
          }
        }
      }
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegularized[i];
    }
  } break;
  default: {
    int i;
    int idxStartIneq;
    obj.nVar = 21;
    obj.mConstr = 41;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegPhaseOne[i];
    }
    i = static_cast<unsigned char>(obj.sizes[0]);
    for (int idx{0}; idx < i; idx++) {
      obj.ATwset[21 * idx + 20] = 0.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = 21;
    obj.lb[20] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    i = obj.nActiveConstr;
    for (int idx{idxStartIneq}; idx <= i; idx++) {
      obj.ATwset[21 * (idx - 1) + 20] = -1.0;
    }
    if (obj.nWConstr[4] > 0) {
      i = static_cast<unsigned char>(obj.sizesNormal[4] + 1);
      for (int idx{0}; idx < i; idx++) {
        obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj.isActiveConstr[obj.isActiveIdx[4] - 2] = false;
  } break;
  }
  obj.probType = PROBLEM_TYPE;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (setProblemType.cpp)
