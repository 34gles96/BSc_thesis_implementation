//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// removeConstr.cpp
//
// Code generation for function 'removeConstr'
//

// Include files
#include "removeConstr.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace WorkingSet {
void removeConstr(h_struct_T &obj, int idx_global)
{
  int TYPE_tmp;
  int i;
  int i1;
  TYPE_tmp = obj.Wid[idx_global - 1] - 1;
  obj.isActiveConstr[(obj.isActiveIdx[TYPE_tmp] +
                      obj.Wlocalidx[idx_global - 1]) -
                     2] = false;
  i = obj.nActiveConstr - 1;
  obj.Wid[idx_global - 1] = obj.Wid[i];
  obj.Wlocalidx[idx_global - 1] = obj.Wlocalidx[i];
  i1 = static_cast<unsigned char>(obj.nVar);
  for (int idx{0}; idx < i1; idx++) {
    obj.ATwset[idx + 21 * (idx_global - 1)] = obj.ATwset[idx + 21 * i];
  }
  obj.bwset[idx_global - 1] = obj.bwset[i];
  obj.nActiveConstr = i;
  obj.nWConstr[TYPE_tmp]--;
}

} // namespace WorkingSet
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (removeConstr.cpp)
