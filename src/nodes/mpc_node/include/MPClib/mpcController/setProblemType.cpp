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
#include "coder_array.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void setProblemType(i_struct_T &obj, int PROBLEM_TYPE)
{
  switch (PROBLEM_TYPE) {
  case 3: {
    int i;
    obj.nVar = obj.nVarOrig;
    obj.mConstr = obj.mConstrOrig;
    if (obj.nWConstr[4] > 0) {
      i = obj.sizesNormal[4];
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
    obj.nVar = obj.nVarOrig + 1;
    obj.mConstr = obj.mConstrOrig + 1;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxPhaseOne[i];
    }
    i = obj.sizes[0];
    for (int idx{0}; idx < i; idx++) {
      obj.ATwset[(obj.nVar + obj.ldA * idx) - 1] = 0.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = obj.nVar;
    obj.lb[obj.nVar - 1] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    i = obj.nActiveConstr;
    for (int idx{idxStartIneq}; idx <= i; idx++) {
      obj.ATwset[(obj.nVar + obj.ldA * (idx - 1)) - 1] = -1.0;
    }
    if (obj.nWConstr[4] > 0) {
      i = obj.sizesNormal[4];
      for (int idx{0}; idx <= i; idx++) {
        obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1] = false;
      }
    }
    obj.isActiveConstr[obj.isActiveIdx[4] - 2] = false;
  } break;
  case 2: {
    int i;
    obj.nVar = obj.nVarMax - 1;
    obj.mConstr = obj.mConstrMax - 1;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegularized[i];
    }
    if (obj.probType != 4) {
      int colOffsetATw;
      int i1;
      int idx;
      int idxStartIneq;
      int offsetIneq_tmp;
      offsetIneq_tmp = obj.nVarOrig + 1;
      i = obj.sizes[0];
      for (int idx_col{0}; idx_col < i; idx_col++) {
        colOffsetATw = obj.ldA * idx_col;
        i1 = obj.nVar;
        for (int idx_row{offsetIneq_tmp}; idx_row <= i1; idx_row++) {
          obj.ATwset[(idx_row + colOffsetATw) - 1] = 0.0;
        }
      }
      colOffsetATw = obj.nVarOrig;
      i = obj.sizesNormal[3] + 1;
      i1 = obj.sizesRegularized[3];
      for (idx = i; idx <= i1; idx++) {
        colOffsetATw++;
        obj.indexLB[idx - 1] = colOffsetATw;
      }
      if (obj.nWConstr[4] > 0) {
        i = obj.sizesRegularized[4];
        for (idx = 0; idx < i; idx++) {
          obj.isActiveConstr[obj.isActiveIdxRegularized[4] + idx] =
              obj.isActiveConstr[(obj.isActiveIdx[4] + idx) - 1];
        }
      }
      i = obj.isActiveIdx[4];
      i1 = obj.isActiveIdxRegularized[4] - 1;
      for (idx = i; idx <= i1; idx++) {
        obj.isActiveConstr[idx - 1] = false;
      }
      i = obj.nVarOrig;
      for (idx = offsetIneq_tmp; idx <= i; idx++) {
        obj.lb[idx - 1] = 0.0;
      }
      idxStartIneq = obj.isActiveIdx[2];
      i = obj.nActiveConstr;
      for (int idx_col{idxStartIneq}; idx_col <= i; idx_col++) {
        colOffsetATw = obj.ldA * (idx_col - 1) - 1;
        if (obj.Wid[idx_col - 1] == 3) {
          i1 = offsetIneq_tmp + obj.Wlocalidx[idx_col - 1];
          idx = i1 - 2;
          for (int idx_row{offsetIneq_tmp}; idx_row <= idx; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
          }
          obj.ATwset[(i1 + colOffsetATw) - 1] = -1.0;
          idx = obj.nVar;
          for (int idx_row{i1}; idx_row <= idx; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
          }
        } else {
          i1 = obj.nVar;
          for (int idx_row{offsetIneq_tmp}; idx_row <= i1; idx_row++) {
            obj.ATwset[idx_row + colOffsetATw] = 0.0;
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
    obj.nVar = obj.nVarMax;
    obj.mConstr = obj.mConstrMax;
    for (i = 0; i < 5; i++) {
      obj.sizes[i] = obj.sizesRegPhaseOne[i];
    }
    for (i = 0; i < 6; i++) {
      obj.isActiveIdx[i] = obj.isActiveIdxRegPhaseOne[i];
    }
    i = obj.sizes[0];
    for (int idx{0}; idx < i; idx++) {
      obj.ATwset[(obj.nVar + obj.ldA * idx) - 1] = 0.0;
    }
    obj.indexLB[obj.sizes[3] - 1] = obj.nVar;
    obj.lb[obj.nVar - 1] = 1.0E-5;
    idxStartIneq = obj.isActiveIdx[2];
    i = obj.nActiveConstr;
    for (int idx{idxStartIneq}; idx <= i; idx++) {
      obj.ATwset[(obj.nVar + obj.ldA * (idx - 1)) - 1] = -1.0;
    }
    if (obj.nWConstr[4] > 0) {
      i = obj.sizesNormal[4];
      for (int idx{0}; idx <= i; idx++) {
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
