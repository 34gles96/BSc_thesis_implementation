//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// RemoveDependentIneq_.cpp
//
// Code generation for function 'RemoveDependentIneq_'
//

// Include files
#include "RemoveDependentIneq_.h"
#include "countsort.h"
#include "mpcController_internal_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void RemoveDependentIneq_(h_struct_T &workingset, d_struct_T &qrmanager,
                          c_struct_T &memspace, double tolfactor)
{
  int nActiveConstr_tmp;
  int nFixedConstr;
  int nVar;
  nActiveConstr_tmp = workingset.nActiveConstr;
  nFixedConstr = workingset.nWConstr[0] + workingset.nWConstr[1];
  nVar = workingset.nVar;
  if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
          workingset.nWConstr[4] >
      0) {
    double tol;
    int idx;
    int idxDiag;
    int nDepIneq;
    tol = tolfactor * static_cast<double>(workingset.nVar) *
          2.2204460492503131E-16;
    for (idx = 0; idx < nFixedConstr; idx++) {
      qrmanager.jpvt[idx] = 1;
    }
    idxDiag = nFixedConstr + 1;
    if (idxDiag <= nActiveConstr_tmp) {
      std::memset(&qrmanager.jpvt[idxDiag + -1], 0,
                  static_cast<unsigned int>((nActiveConstr_tmp - idxDiag) + 1) *
                      sizeof(int));
    }
    for (int idx_col{0}; idx_col < nActiveConstr_tmp; idx_col++) {
      nDepIneq = 41 * idx_col;
      idx = 21 * idx_col;
      idxDiag = static_cast<unsigned char>(nVar);
      for (int k{0}; k < idxDiag; k++) {
        qrmanager.QR[nDepIneq + k] = workingset.ATwset[idx + k];
      }
    }
    if (workingset.nVar * workingset.nActiveConstr == 0) {
      qrmanager.mrows = workingset.nVar;
      qrmanager.ncols = workingset.nActiveConstr;
      qrmanager.minRowCol = 0;
    } else {
      qrmanager.usedPivoting = true;
      qrmanager.mrows = workingset.nVar;
      qrmanager.ncols = workingset.nActiveConstr;
      idxDiag = workingset.nVar;
      nDepIneq = workingset.nActiveConstr;
      if (idxDiag <= nDepIneq) {
        nDepIneq = idxDiag;
      }
      qrmanager.minRowCol = nDepIneq;
      internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                               workingset.nActiveConstr, qrmanager.jpvt,
                               qrmanager.tau);
    }
    nDepIneq = 0;
    for (idx = workingset.nActiveConstr - 1; idx + 1 > nVar; idx--) {
      nDepIneq++;
      memspace.workspace_int[nDepIneq - 1] = qrmanager.jpvt[idx];
    }
    if (idx + 1 <= workingset.nVar) {
      idxDiag = idx + 41 * idx;
      while ((idx + 1 > nFixedConstr) &&
             (std::abs(qrmanager.QR[idxDiag]) < tol)) {
        nDepIneq++;
        memspace.workspace_int[nDepIneq - 1] = qrmanager.jpvt[idx];
        idx--;
        idxDiag -= 42;
      }
    }
    utils::countsort(memspace.workspace_int, nDepIneq, memspace.workspace_sort,
                     nFixedConstr + 1, workingset.nActiveConstr);
    for (idx = nDepIneq; idx >= 1; idx--) {
      WorkingSet::removeConstr(workingset, memspace.workspace_int[idx - 1]);
    }
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (RemoveDependentIneq_.cpp)
