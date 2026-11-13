//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// PresolveWorkingSet.cpp
//
// Code generation for function 'PresolveWorkingSet'
//

// Include files
#include "PresolveWorkingSet.h"
#include "RemoveDependentIneq_.h"
#include "computeQ_.h"
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpcController_internal_types.h"
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
void PresolveWorkingSet(g_struct_T &solution, c_struct_T &memspace,
                        h_struct_T &workingset, d_struct_T &qrmanager)
{
  double tol;
  int idxDiag;
  int ix;
  int ix0;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  solution.state = 82;
  mWorkingFixed = workingset.nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset.nWConstr[0] + workingset.nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    int i;
    int i1;
    int u0;
    i = static_cast<unsigned char>(workingset.nVar);
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (int idx_col{0}; idx_col < i; idx_col++) {
        qrmanager.QR[ix + 41 * idx_col] = workingset.ATwset[idx_col + 21 * ix];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset.nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    std::memset(&qrmanager.jpvt[0], 0,
                static_cast<unsigned int>(i) * sizeof(int));
    i1 = mTotalWorkingEq_tmp_tmp * workingset.nVar;
    if (i1 == 0) {
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      qrmanager.minRowCol = 0;
    } else {
      qrmanager.usedPivoting = true;
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      idxDiag = workingset.nVar;
      if (mTotalWorkingEq_tmp_tmp <= idxDiag) {
        idxDiag = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager.minRowCol = idxDiag;
      internal::lapack::xgeqp3(qrmanager.QR, mTotalWorkingEq_tmp_tmp,
                               workingset.nVar, qrmanager.jpvt, qrmanager.tau);
    }
    tol = 100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
    u0 = workingset.nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = u0 + 41 * (u0 - 1);
    while ((idxDiag > 0) && (std::abs(qrmanager.QR[idxDiag - 1]) < tol)) {
      idxDiag -= 42;
      nDepInd++;
    }
    if (nDepInd > 0) {
      boolean_T exitg1;
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      idxDiag = 0;
      exitg1 = false;
      while ((!exitg1) && (idxDiag <= nDepInd - 1)) {
        double qtb;
        ix = 41 * ((mTotalWorkingEq_tmp_tmp - idxDiag) - 1);
        qtb = 0.0;
        for (int k{0}; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager.Q[ix + k] * workingset.bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idxDiag++;
        }
      }
    }
    if (nDepInd > 0) {
      for (int idx_col{0}; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idxDiag = 41 * idx_col;
        ix0 = 21 * idx_col;
        for (int k{0}; k < i; k++) {
          qrmanager.QR[idxDiag + k] = workingset.ATwset[ix0 + k];
        }
      }
      for (idxDiag = 0; idxDiag < mWorkingFixed; idxDiag++) {
        qrmanager.jpvt[idxDiag] = 1;
      }
      i = workingset.nWConstr[0] + 1;
      if (i <= mTotalWorkingEq_tmp_tmp) {
        std::memset(
            &qrmanager.jpvt[i + -1], 0,
            static_cast<unsigned int>((mTotalWorkingEq_tmp_tmp - i) + 1) *
                sizeof(int));
      }
      if (i1 == 0) {
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = true;
        qrmanager.mrows = workingset.nVar;
        qrmanager.ncols = mTotalWorkingEq_tmp_tmp;
        qrmanager.minRowCol = u0;
        internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                 mTotalWorkingEq_tmp_tmp, qrmanager.jpvt,
                                 qrmanager.tau);
      }
      for (idxDiag = 0; idxDiag < nDepInd; idxDiag++) {
        memspace.workspace_int[idxDiag] =
            qrmanager.jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idxDiag];
      }
      utils::countsort(memspace.workspace_int, nDepInd, memspace.workspace_sort,
                       1, mTotalWorkingEq_tmp_tmp);
      for (idxDiag = nDepInd; idxDiag >= 1; idxDiag--) {
        i = memspace.workspace_int[idxDiag - 1];
        if (i <= mTotalWorkingEq_tmp_tmp) {
          if ((workingset.nActiveConstr == mTotalWorkingEq_tmp_tmp) ||
              (i == mTotalWorkingEq_tmp_tmp)) {
            workingset.mEqRemoved++;
            // A check that is always false is detected at compile-time.
            // Eliminating code that follows.
          } else {
            workingset.mEqRemoved++;
            // A check that is always false is detected at compile-time.
            // Eliminating code that follows.
          }
        }
      }
    }
  }
  if ((nDepInd != -1) && (workingset.nActiveConstr <= 41)) {
    boolean_T guard1;
    boolean_T okWorkingSet;
    RemoveDependentIneq_(workingset, qrmanager, memspace, 100.0);
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace.workspace_double, solution.xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      RemoveDependentIneq_(workingset, qrmanager, memspace, 1000.0);
      okWorkingSet = feasibleX0ForWorkingSet(
          memspace.workspace_double, solution.xstar, workingset, qrmanager);
      if (!okWorkingSet) {
        solution.state = -7;
      } else {
        guard1 = true;
      }
    } else {
      guard1 = true;
    }
    if (guard1 &&
        (workingset.nWConstr[0] + workingset.nWConstr[1] == workingset.nVar)) {
      tol = WorkingSet::maxConstraintViolation(workingset, solution.xstar);
      if (tol > 1.0E-6) {
        solution.state = -2;
      }
    }
  } else {
    solution.state = -3;
    idxDiag = mTotalWorkingEq_tmp_tmp + 1;
    ix = workingset.nActiveConstr;
    for (ix0 = idxDiag; ix0 <= ix; ix0++) {
      workingset
          .isActiveConstr[(workingset.isActiveIdx[workingset.Wid[ix0 - 1] - 1] +
                           workingset.Wlocalidx[ix0 - 1]) -
                          2] = false;
    }
    workingset.nWConstr[2] = 0;
    workingset.nWConstr[3] = 0;
    workingset.nWConstr[4] = 0;
    workingset.nActiveConstr = mTotalWorkingEq_tmp_tmp;
  }
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (PresolveWorkingSet.cpp)
