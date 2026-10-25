//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// feasibleX0ForWorkingSet.cpp
//
// Code generation for function 'feasibleX0ForWorkingSet'
//

// Include files
#include "feasibleX0ForWorkingSet.h"
#include "computeQ_.h"
#include "factorQR.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(array<double, 2U> &workspace,
                                  array<double, 1U> &xCurrent,
                                  const i_struct_T &workingset,
                                  d_struct_T &qrmanager)
{
  array<double, 2U> B;
  int mWConstr;
  int nVar;
  boolean_T nonDegenerateWset;
  mWConstr = workingset.nActiveConstr;
  nVar = workingset.nVar;
  nonDegenerateWset = true;
  if (mWConstr != 0) {
    double c;
    int br;
    int i;
    int i1;
    int iAcol;
    int idx;
    int iy;
    int jBcol;
    int mFixed;
    for (idx = 0; idx < mWConstr; idx++) {
      c = workingset.bwset[idx];
      workspace[idx] = c;
      workspace[idx + workspace.size(0)] = c;
    }
    iAcol = workingset.ldA;
    if ((nVar != 0) && (mWConstr != 0)) {
      iy = 0;
      i = workingset.ldA * (mWConstr - 1) + 1;
      for (jBcol = 1; iAcol < 0 ? jBcol >= i : jBcol <= i; jBcol += iAcol) {
        c = 0.0;
        i1 = (jBcol + nVar) - 1;
        for (mFixed = jBcol; mFixed <= i1; mFixed++) {
          c += workingset.ATwset[mFixed - 1] * xCurrent[mFixed - jBcol];
        }
        workspace[iy] = workspace[iy] - c;
        iy++;
      }
    }
    if (mWConstr >= nVar) {
      int ldq;
      int ldw;
      for (iy = 0; iy < nVar; iy++) {
        iAcol = qrmanager.ldq * iy;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager.QR[jBcol + iAcol] =
              workingset.ATwset[iy + workingset.ldA * jBcol];
        }
      }
      if (mWConstr * nVar == 0) {
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        qrmanager.minRowCol = 0;
      } else {
        qrmanager.usedPivoting = false;
        qrmanager.mrows = mWConstr;
        qrmanager.ncols = nVar;
        for (idx = 0; idx < nVar; idx++) {
          qrmanager.jpvt[idx] = idx + 1;
        }
        if (mWConstr <= nVar) {
          i = mWConstr;
        } else {
          i = nVar;
        }
        qrmanager.minRowCol = i;
        B.set_size(qrmanager.QR.size(0), qrmanager.QR.size(1));
        iAcol = qrmanager.QR.size(0) * qrmanager.QR.size(1);
        for (i1 = 0; i1 < iAcol; i1++) {
          B[i1] = qrmanager.QR[i1];
        }
        iAcol = qrmanager.QR.size(0);
        iy = qrmanager.QR.size(1);
        if (iAcol <= iy) {
          iy = iAcol;
        }
        qrmanager.tau.set_size(iy);
        for (i1 = 0; i1 < iy; i1++) {
          qrmanager.tau[i1] = 0.0;
        }
        if (i >= 1) {
          internal::reflapack::qrf(B, mWConstr, nVar, i, qrmanager.tau);
        }
        qrmanager.QR.set_size(B.size(0), B.size(1));
        iAcol = B.size(0) * B.size(1);
        for (i = 0; i < iAcol; i++) {
          qrmanager.QR[i] = B[i];
        }
      }
      QRManager::computeQ_(qrmanager, qrmanager.mrows);
      ldq = qrmanager.ldq;
      ldw = workspace.size(0);
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      if (nVar != 0) {
        for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
          i = jBcol + 1;
          i1 = jBcol + nVar;
          for (int k{i}; k <= i1; k++) {
            workspace[k - 1] = 0.0;
          }
        }
        br = -1;
        for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
          iAcol = -1;
          i = jBcol + 1;
          i1 = jBcol + nVar;
          for (int k{i}; k <= i1; k++) {
            c = 0.0;
            for (iy = 0; iy < mWConstr; iy++) {
              c += qrmanager.Q[(iy + iAcol) + 1] * B[(iy + br) + 1];
            }
            workspace[k - 1] = workspace[k - 1] + c;
            iAcol += ldq;
          }
          br += ldw;
        }
      }
      for (mFixed = 0; mFixed < 2; mFixed++) {
        jBcol = ldw * mFixed - 1;
        for (int k{nVar}; k >= 1; k--) {
          iy = ldq * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager.QR[k + iy];
            for (idx = 0; idx <= k - 2; idx++) {
              i1 = (idx + jBcol) + 1;
              workspace[i1] =
                  workspace[i1] - workspace[i] * qrmanager.QR[(idx + iy) + 1];
            }
          }
        }
      }
    } else {
      int ldq;
      int ldw;
      QRManager::factorQR(qrmanager, workingset.ATwset, nVar, mWConstr,
                          workingset.ldA);
      QRManager::computeQ_(qrmanager, qrmanager.minRowCol);
      ldq = qrmanager.ldq;
      ldw = workspace.size(0);
      for (mFixed = 0; mFixed < 2; mFixed++) {
        jBcol = ldw * mFixed;
        for (idx = 0; idx < mWConstr; idx++) {
          iAcol = ldq * idx;
          iy = idx + jBcol;
          c = workspace[iy];
          for (int k{0}; k < idx; k++) {
            c -= qrmanager.QR[k + iAcol] * workspace[k + jBcol];
          }
          workspace[iy] = c / qrmanager.QR[idx + iAcol];
        }
      }
      B.set_size(workspace.size(0), workspace.size(1));
      iAcol = workspace.size(0) * workspace.size(1);
      for (i = 0; i < iAcol; i++) {
        B[i] = workspace[i];
      }
      if (nVar != 0) {
        for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
          i = jBcol + 1;
          i1 = jBcol + nVar;
          for (int k{i}; k <= i1; k++) {
            workspace[k - 1] = 0.0;
          }
        }
        br = 0;
        for (jBcol = 0; ldw < 0 ? jBcol >= ldw : jBcol <= ldw; jBcol += ldw) {
          iAcol = -1;
          i = br + 1;
          i1 = br + mWConstr;
          for (idx = i; idx <= i1; idx++) {
            iy = jBcol + 1;
            mFixed = jBcol + nVar;
            for (int k{iy}; k <= mFixed; k++) {
              workspace[k - 1] = workspace[k - 1] +
                                 B[idx - 1] * qrmanager.Q[(iAcol + k) - jBcol];
            }
            iAcol += ldq;
          }
          br += ldw;
        }
      }
    }
    idx = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (idx <= nVar - 1) {
        c = workspace[idx];
        if (std::isinf(c) || std::isnan(c)) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[idx + workspace.size(0)];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            idx++;
          }
        }
      } else {
        double constrViolation_basicX;
        if (nVar >= 1) {
          iAcol = nVar - 1;
          for (int k{0}; k <= iAcol; k++) {
            workspace[k] = workspace[k] + xCurrent[k];
          }
        }
        br = workspace.size(0) - 1;
        iAcol = workingset.sizes[3];
        jBcol = workingset.sizes[4];
        mFixed = workingset.sizes[0];
        c = 0.0;
        if (workingset.sizes[3] > 0) {
          for (idx = 0; idx < iAcol; idx++) {
            iy = workingset.indexLB[idx] - 1;
            c = std::fmax(c, -workspace[iy] - workingset.lb[iy]);
          }
        }
        if (workingset.sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            iAcol = workingset.indexUB[idx] - 1;
            c = std::fmax(c, workspace[iAcol] - workingset.ub[iAcol]);
          }
        }
        if (workingset.sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            c = std::fmax(
                c, std::abs(workspace[workingset.indexFixed[idx] - 1] -
                            workingset.ub[workingset.indexFixed[idx] - 1]));
          }
        }
        iAcol = workingset.sizes[3];
        jBcol = workingset.sizes[4];
        mFixed = workingset.sizes[0];
        constrViolation_basicX = 0.0;
        if (workingset.sizes[3] > 0) {
          for (idx = 0; idx < iAcol; idx++) {
            constrViolation_basicX =
                std::fmax(constrViolation_basicX,
                          -workspace[br + workingset.indexLB[idx]] -
                              workingset.lb[workingset.indexLB[idx] - 1]);
          }
        }
        if (workingset.sizes[4] > 0) {
          for (idx = 0; idx < jBcol; idx++) {
            constrViolation_basicX =
                std::fmax(constrViolation_basicX,
                          workspace[br + workingset.indexUB[idx]] -
                              workingset.ub[workingset.indexUB[idx] - 1]);
          }
        }
        if (workingset.sizes[0] > 0) {
          for (idx = 0; idx < mFixed; idx++) {
            constrViolation_basicX = std::fmax(
                constrViolation_basicX,
                std::abs(workspace[br + workingset.indexFixed[idx]] -
                         workingset.ub[workingset.indexFixed[idx] - 1]));
          }
        }
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          for (int k{0}; k < nVar; k++) {
            xCurrent[k] = workspace[k];
          }
        } else {
          for (int k{0}; k < nVar; k++) {
            xCurrent[k] = workspace[(br + k) + 1];
          }
        }
        exitg1 = 1;
      }
    } while (exitg1 == 0);
  }
  return nonDegenerateWset;
}

} // namespace initialize
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (feasibleX0ForWorkingSet.cpp)
