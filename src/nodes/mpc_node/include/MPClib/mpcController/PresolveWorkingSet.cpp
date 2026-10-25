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
#include "computeQ_.h"
#include "countsort.h"
#include "feasibleX0ForWorkingSet.h"
#include "maxConstraintViolation.h"
#include "mpcController_internal_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "xgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
void PresolveWorkingSet(h_struct_T &solution, g_struct_T &memspace,
                        i_struct_T &workingset, d_struct_T &qrmanager)
{
  double tol;
  int i;
  int idx;
  int idxDiag;
  int ix;
  int mTotalWorkingEq_tmp_tmp;
  int mWorkingFixed;
  int nDepInd;
  int nVar;
  int u0;
  solution.state = 82;
  nVar = workingset.nVar - 1;
  mWorkingFixed = workingset.nWConstr[0];
  mTotalWorkingEq_tmp_tmp = workingset.nWConstr[0] + workingset.nWConstr[1];
  nDepInd = 0;
  if (mTotalWorkingEq_tmp_tmp > 0) {
    for (ix = 0; ix < mTotalWorkingEq_tmp_tmp; ix++) {
      for (int idx_col{0}; idx_col <= nVar; idx_col++) {
        qrmanager.QR[ix + qrmanager.ldq * idx_col] =
            workingset.ATwset[idx_col + workingset.ldA * ix];
      }
    }
    nDepInd = mTotalWorkingEq_tmp_tmp - workingset.nVar;
    if (nDepInd <= 0) {
      nDepInd = 0;
    }
    for (idx = 0; idx <= nVar; idx++) {
      qrmanager.jpvt[idx] = 0;
    }
    i = mTotalWorkingEq_tmp_tmp * workingset.nVar;
    if (i == 0) {
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      qrmanager.minRowCol = 0;
    } else {
      qrmanager.usedPivoting = true;
      qrmanager.mrows = mTotalWorkingEq_tmp_tmp;
      qrmanager.ncols = workingset.nVar;
      ix = workingset.nVar;
      if (mTotalWorkingEq_tmp_tmp <= ix) {
        ix = mTotalWorkingEq_tmp_tmp;
      }
      qrmanager.minRowCol = ix;
      internal::lapack::xgeqp3(qrmanager.QR, mTotalWorkingEq_tmp_tmp,
                               workingset.nVar, qrmanager.jpvt, qrmanager.tau);
    }
    tol = 100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
    u0 = workingset.nVar;
    if (u0 > mTotalWorkingEq_tmp_tmp) {
      u0 = mTotalWorkingEq_tmp_tmp;
    }
    idxDiag = u0 + qrmanager.ldq * (u0 - 1);
    while ((idxDiag > 0) && (std::abs(qrmanager.QR[idxDiag - 1]) < tol)) {
      idxDiag = (idxDiag - qrmanager.ldq) - 1;
      nDepInd++;
    }
    if (nDepInd > 0) {
      boolean_T exitg1;
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= nDepInd - 1)) {
        double qtb;
        ix = qrmanager.ldq * ((mTotalWorkingEq_tmp_tmp - idx) - 1);
        qtb = 0.0;
        for (int k{0}; k < mTotalWorkingEq_tmp_tmp; k++) {
          qtb += qrmanager.Q[ix + k] * workingset.bwset[k];
        }
        if (std::abs(qtb) >= tol) {
          nDepInd = -1;
          exitg1 = true;
        } else {
          idx++;
        }
      }
    }
    if (nDepInd > 0) {
      for (int idx_col{0}; idx_col < mTotalWorkingEq_tmp_tmp; idx_col++) {
        idx = qrmanager.ldq * idx_col;
        idxDiag = workingset.ldA * idx_col;
        for (int k{0}; k <= nVar; k++) {
          qrmanager.QR[idx + k] = workingset.ATwset[idxDiag + k];
        }
      }
      for (idx = 0; idx < mWorkingFixed; idx++) {
        qrmanager.jpvt[idx] = 1;
      }
      mWorkingFixed = workingset.nWConstr[0] + 1;
      for (idx = mWorkingFixed; idx <= mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager.jpvt[idx - 1] = 0;
      }
      if (i == 0) {
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
      for (idx = 0; idx < nDepInd; idx++) {
        memspace.workspace_int[idx] =
            qrmanager.jpvt[(mTotalWorkingEq_tmp_tmp - nDepInd) + idx];
      }
      utils::countsort(memspace.workspace_int, nDepInd, memspace.workspace_sort,
                       1, mTotalWorkingEq_tmp_tmp);
      for (idx = nDepInd; idx >= 1; idx--) {
        i = memspace.workspace_int[idx - 1];
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
  if ((nDepInd != -1) && (workingset.nActiveConstr <= qrmanager.ldq)) {
    boolean_T guard1;
    boolean_T okWorkingSet;
    ix = workingset.nActiveConstr;
    nVar = workingset.nVar;
    if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
            workingset.nWConstr[4] >
        0) {
      tol =
          100.0 * static_cast<double>(workingset.nVar) * 2.2204460492503131E-16;
      for (idx = 0; idx < mTotalWorkingEq_tmp_tmp; idx++) {
        qrmanager.jpvt[idx] = 1;
      }
      i = mTotalWorkingEq_tmp_tmp + 1;
      for (idx = i; idx <= ix; idx++) {
        qrmanager.jpvt[idx - 1] = 0;
      }
      i = workingset.nActiveConstr;
      for (int idx_col{0}; idx_col < i; idx_col++) {
        idx = qrmanager.ldq * idx_col;
        idxDiag = workingset.ldA * idx_col;
        for (int k{0}; k < nVar; k++) {
          qrmanager.QR[idx + k] = workingset.ATwset[idxDiag + k];
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
        u0 = workingset.nVar;
        ix = workingset.nActiveConstr;
        if (u0 <= ix) {
          ix = u0;
        }
        qrmanager.minRowCol = ix;
        internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                 workingset.nActiveConstr, qrmanager.jpvt,
                                 qrmanager.tau);
      }
      ix = 0;
      for (idx = workingset.nActiveConstr - 1; idx + 1 > nVar; idx--) {
        ix++;
        memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx];
      }
      if (idx + 1 <= workingset.nVar) {
        idxDiag = idx + qrmanager.ldq * idx;
        while ((idx + 1 > mTotalWorkingEq_tmp_tmp) &&
               (std::abs(qrmanager.QR[idxDiag]) < tol)) {
          ix++;
          memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx];
          idx--;
          idxDiag = (idxDiag - qrmanager.ldq) - 1;
        }
      }
      utils::countsort(memspace.workspace_int, ix, memspace.workspace_sort,
                       mTotalWorkingEq_tmp_tmp + 1, workingset.nActiveConstr);
      for (idx = ix; idx >= 1; idx--) {
        WorkingSet::removeConstr(workingset, memspace.workspace_int[idx - 1]);
      }
    }
    okWorkingSet = feasibleX0ForWorkingSet(
        memspace.workspace_double, solution.xstar, workingset, qrmanager);
    guard1 = false;
    if (!okWorkingSet) {
      ix = workingset.nActiveConstr;
      i = workingset.nWConstr[0] + workingset.nWConstr[1];
      nVar = workingset.nVar;
      if ((workingset.nWConstr[2] + workingset.nWConstr[3]) +
              workingset.nWConstr[4] >
          0) {
        tol = 1000.0 * static_cast<double>(workingset.nVar) *
              2.2204460492503131E-16;
        for (idx = 0; idx < i; idx++) {
          qrmanager.jpvt[idx] = 1;
        }
        mWorkingFixed = i + 1;
        for (idx = mWorkingFixed; idx <= ix; idx++) {
          qrmanager.jpvt[idx - 1] = 0;
        }
        mWorkingFixed = workingset.nActiveConstr;
        for (int idx_col{0}; idx_col < mWorkingFixed; idx_col++) {
          idx = qrmanager.ldq * idx_col;
          idxDiag = workingset.ldA * idx_col;
          for (int k{0}; k < nVar; k++) {
            qrmanager.QR[idx + k] = workingset.ATwset[idxDiag + k];
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
          u0 = workingset.nVar;
          ix = workingset.nActiveConstr;
          if (u0 <= ix) {
            ix = u0;
          }
          qrmanager.minRowCol = ix;
          internal::lapack::xgeqp3(qrmanager.QR, workingset.nVar,
                                   workingset.nActiveConstr, qrmanager.jpvt,
                                   qrmanager.tau);
        }
        ix = 0;
        for (idx = workingset.nActiveConstr - 1; idx + 1 > nVar; idx--) {
          ix++;
          memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx];
        }
        if (idx + 1 <= workingset.nVar) {
          idxDiag = idx + qrmanager.ldq * idx;
          while ((idx + 1 > i) && (std::abs(qrmanager.QR[idxDiag]) < tol)) {
            ix++;
            memspace.workspace_int[ix - 1] = qrmanager.jpvt[idx];
            idx--;
            idxDiag = (idxDiag - qrmanager.ldq) - 1;
          }
        }
        utils::countsort(memspace.workspace_int, ix, memspace.workspace_sort,
                         i + 1, workingset.nActiveConstr);
        for (idx = ix; idx >= 1; idx--) {
          WorkingSet::removeConstr(workingset, memspace.workspace_int[idx - 1]);
        }
      }
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
    ix = mTotalWorkingEq_tmp_tmp + 1;
    idxDiag = workingset.nActiveConstr;
    for (idx = ix; idx <= idxDiag; idx++) {
      workingset
          .isActiveConstr[(workingset.isActiveIdx[workingset.Wid[idx - 1] - 1] +
                           workingset.Wlocalidx[idx - 1]) -
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
