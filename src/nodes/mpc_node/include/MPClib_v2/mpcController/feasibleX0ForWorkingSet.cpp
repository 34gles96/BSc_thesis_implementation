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
#include "maxConstraintViolation.h"
#include "mpcController_internal_types.h"
#include "mpcController_rtwutil.h"
#include "rt_nonfinite.h"
#include "xzgeqp3.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
boolean_T feasibleX0ForWorkingSet(double workspace[861], double xCurrent[21],
                                  const h_struct_T &workingset,
                                  d_struct_T &qrmanager)
{
  double B[861];
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
    for (iAcol = 0; iAcol < mWConstr; iAcol++) {
      c = workingset.bwset[iAcol];
      workspace[iAcol] = c;
      workspace[iAcol + 41] = c;
    }
    if (mWConstr != 0) {
      i = 21 * (mWConstr - 1) + 1;
      for (iAcol = 1; iAcol <= i; iAcol += 21) {
        c = 0.0;
        i1 = (iAcol + nVar) - 1;
        for (br = iAcol; br <= i1; br++) {
          c += workingset.ATwset[br - 1] * xCurrent[br - iAcol];
        }
        i1 = div_nde_s32_floor(iAcol - 1, 21);
        workspace[i1] -= c;
      }
    }
    if (mWConstr >= nVar) {
      int jBcol;
      i = static_cast<unsigned char>(nVar);
      qrmanager.usedPivoting = false;
      qrmanager.mrows = mWConstr;
      qrmanager.ncols = nVar;
      for (br = 0; br < i; br++) {
        iAcol = 41 * br;
        for (jBcol = 0; jBcol < mWConstr; jBcol++) {
          qrmanager.QR[jBcol + iAcol] = workingset.ATwset[br + 21 * jBcol];
        }
        qrmanager.jpvt[br] = br + 1;
      }
      if (mWConstr <= nVar) {
        i = mWConstr;
      } else {
        i = nVar;
      }
      qrmanager.minRowCol = i;
      std::memset(&qrmanager.tau[0], 0, 41U * sizeof(double));
      if (i >= 1) {
        internal::reflapack::qrf(qrmanager.QR, mWConstr, nVar, i,
                                 qrmanager.tau);
      }
      QRManager::computeQ_(qrmanager, mWConstr);
      std::copy(&workspace[0], &workspace[861], &B[0]);
      for (int b_i{0}; b_i <= 41; b_i += 41) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0,
                      static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
        }
      }
      br = -1;
      for (int b_i{0}; b_i <= 41; b_i += 41) {
        jBcol = -1;
        i = b_i + 1;
        i1 = b_i + nVar;
        for (int k{i}; k <= i1; k++) {
          c = 0.0;
          for (iAcol = 0; iAcol < mWConstr; iAcol++) {
            c += qrmanager.Q[(iAcol + jBcol) + 1] * B[(iAcol + br) + 1];
          }
          workspace[k - 1] += c;
          jBcol += 41;
        }
        br += 41;
      }
      for (int j{0}; j < 2; j++) {
        jBcol = 41 * j - 1;
        for (int k{nVar}; k >= 1; k--) {
          iAcol = 41 * (k - 1) - 1;
          i = k + jBcol;
          c = workspace[i];
          if (c != 0.0) {
            workspace[i] = c / qrmanager.QR[k + iAcol];
            i1 = static_cast<unsigned char>(k - 1);
            for (int b_i{0}; b_i < i1; b_i++) {
              int i2;
              i2 = (b_i + jBcol) + 1;
              workspace[i2] -= workspace[i] * qrmanager.QR[(b_i + iAcol) + 1];
            }
          }
        }
      }
    } else {
      int jBcol;
      QRManager::factorQR(qrmanager, workingset.ATwset, nVar, mWConstr);
      QRManager::computeQ_(qrmanager, qrmanager.minRowCol);
      for (int j{0}; j < 2; j++) {
        jBcol = 41 * j;
        for (int b_i{0}; b_i < mWConstr; b_i++) {
          iAcol = 41 * b_i;
          br = b_i + jBcol;
          c = workspace[br];
          i = static_cast<unsigned char>(b_i);
          for (int k{0}; k < i; k++) {
            c -= qrmanager.QR[k + iAcol] * workspace[k + jBcol];
          }
          workspace[br] = c / qrmanager.QR[b_i + iAcol];
        }
      }
      std::copy(&workspace[0], &workspace[861], &B[0]);
      for (int b_i{0}; b_i <= 41; b_i += 41) {
        i = b_i + 1;
        i1 = b_i + nVar;
        if (i <= i1) {
          std::memset(&workspace[i + -1], 0,
                      static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
        }
      }
      br = 0;
      for (int b_i{0}; b_i <= 41; b_i += 41) {
        jBcol = -1;
        i = br + 1;
        i1 = br + mWConstr;
        for (int j{i}; j <= i1; j++) {
          int i2;
          i2 = b_i + 1;
          iAcol = b_i + nVar;
          for (int k{i2}; k <= iAcol; k++) {
            workspace[k - 1] += B[j - 1] * qrmanager.Q[(jBcol + k) - b_i];
          }
          jBcol += 41;
        }
        br += 41;
      }
    }
    iAcol = 0;
    int exitg1;
    do {
      exitg1 = 0;
      if (iAcol <= static_cast<unsigned char>(nVar) - 1) {
        if (std::isinf(workspace[iAcol]) || std::isnan(workspace[iAcol])) {
          nonDegenerateWset = false;
          exitg1 = 1;
        } else {
          c = workspace[iAcol + 41];
          if (std::isinf(c) || std::isnan(c)) {
            nonDegenerateWset = false;
            exitg1 = 1;
          } else {
            iAcol++;
          }
        }
      } else {
        double constrViolation_basicX;
        iAcol = nVar - 1;
        for (int k{0}; k <= iAcol; k++) {
          workspace[k] += xCurrent[k];
        }
        c = WorkingSet::maxConstraintViolation(workingset, workspace, 1);
        constrViolation_basicX =
            WorkingSet::maxConstraintViolation(workingset, workspace, 42);
        if ((c <= 2.2204460492503131E-16) || (c < constrViolation_basicX)) {
          i = static_cast<unsigned char>(nVar);
          std::copy(&workspace[0], &workspace[i], &xCurrent[0]);
        } else {
          i = static_cast<unsigned char>(nVar);
          std::copy(&workspace[41], &workspace[41 + i], &xCurrent[0]);
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
