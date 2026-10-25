//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// test_exit.cpp
//
// Code generation for function 'test_exit'
//

// Include files
#include "test_exit.h"
#include "computeComplError.h"
#include "computeQ_.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xgeqp3.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(b_struct_T &Flags, g_struct_T &memspace,
                 struct_T &MeritFunction, const i_struct_T &WorkingSet,
                 h_struct_T &b_TrialState, d_struct_T &b_QRManager,
                 const array<double, 1U> &lb, const array<double, 1U> &ub,
                 int runTimeOptions_MaxFunctionEvaluations)
{
  const array<int, 1U> *finiteFixed;
  const array<int, 1U> *finiteLB;
  const array<int, 1U> *finiteUB;
  array<double, 2U> *b_workspace;
  array<double, 1U> *lambda;
  array<double, 1U> *workspace;
  double optimRelativeFactor;
  double s;
  double smax;
  int iL0;
  int idx;
  int idxFiniteLB;
  int k;
  int mFixed;
  int mLB;
  int mLambda;
  int mUB;
  int nActiveConstr;
  int nVar;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T isFeasible;
  nVar = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mLambda =
      ((WorkingSet.sizes[0] + WorkingSet.sizes[3]) + WorkingSet.sizes[4]) - 1;
  for (k = 0; k <= mLambda; k++) {
    b_TrialState.lambdaStopTest[k] = b_TrialState.lambdasqp[k];
  }
  workspace = &b_TrialState.gradLag;
  idxFiniteLB = WorkingSet.nVar;
  finiteFixed = &WorkingSet.indexFixed;
  iL0 = WorkingSet.sizes[0];
  finiteLB = &WorkingSet.indexLB;
  nActiveConstr = WorkingSet.sizes[3];
  finiteUB = &WorkingSet.indexUB;
  k = WorkingSet.sizes[4];
  lambda = &b_TrialState.lambdaStopTest;
  for (int i{0}; i < idxFiniteLB; i++) {
    (*workspace)[i] = b_TrialState.grad[i];
  }
  for (idx = 0; idx < iL0; idx++) {
    (*workspace)[(*finiteFixed)[idx] - 1] =
        (*workspace)[(*finiteFixed)[idx] - 1] + (*lambda)[idx];
  }
  for (idx = 0; idx < nActiveConstr; idx++) {
    (*workspace)[(*finiteLB)[idx] - 1] =
        (*workspace)[(*finiteLB)[idx] - 1] - (*lambda)[iL0 + idx];
  }
  if (nActiveConstr - 1 >= 0) {
    iL0 += nActiveConstr;
  }
  for (idx = 0; idx < k; idx++) {
    (*workspace)[(*finiteUB)[idx] - 1] =
        (*workspace)[(*finiteUB)[idx] - 1] + (*lambda)[iL0 + idx];
  }
  if (WorkingSet.nVar < 1) {
    iL0 = 0;
  } else {
    iL0 = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = std::abs(b_TrialState.grad[k - 1]);
        if (s > smax) {
          iL0 = k;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor = std::fmax(1.0, std::abs(b_TrialState.grad[iL0 - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  smax = 0.0;
  for (idx = 0; idx < mLB; idx++) {
    idxFiniteLB = WorkingSet.indexLB[idx] - 1;
    smax =
        std::fmax(smax, lb[idxFiniteLB] - b_TrialState.xstarsqp[idxFiniteLB]);
  }
  for (idx = 0; idx < mUB; idx++) {
    idxFiniteLB = WorkingSet.indexUB[idx] - 1;
    smax =
        std::fmax(smax, b_TrialState.xstarsqp[idxFiniteLB] - ub[idxFiniteLB]);
  }
  MeritFunction.nlpPrimalFeasError = smax;
  if (b_TrialState.sqpIterations == 0) {
    MeritFunction.feasRelativeFactor = std::fmax(1.0, smax);
  }
  isFeasible = (smax <= 1.0E-6 * MeritFunction.feasRelativeFactor);
  idxFiniteLB = WorkingSet.nVar;
  workspace = &b_TrialState.gradLag;
  dxTooSmall = true;
  smax = 0.0;
  idx = 0;
  exitg1 = false;
  while ((!exitg1) && (idx <= idxFiniteLB - 1)) {
    dxTooSmall =
        ((!std::isinf((*workspace)[idx])) && (!std::isnan((*workspace)[idx])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs((*workspace)[idx]));
      idx++;
    }
  }
  Flags.gradOK = dxTooSmall;
  MeritFunction.nlpDualFeasError = smax;
  if (!Flags.gradOK) {
    Flags.done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = stopping::computeComplError(
        b_TrialState.xstarsqp, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
        WorkingSet.indexUB, WorkingSet.sizes[4], ub,
        b_TrialState.lambdaStopTest, WorkingSet.sizes[0] + 1);
    smax =
        std::fmax(MeritFunction.nlpDualFeasError, MeritFunction.nlpComplError);
    MeritFunction.firstOrderOpt = smax;
    if (b_TrialState.sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      b_workspace = &memspace.workspace_double;
      idxFiniteLB = WorkingSet.nVar;
      finiteFixed = &WorkingSet.indexFixed;
      iL0 = WorkingSet.sizes[0];
      finiteLB = &WorkingSet.indexLB;
      mLB = WorkingSet.sizes[3];
      finiteUB = &WorkingSet.indexUB;
      mUB = WorkingSet.sizes[4];
      lambda = &b_TrialState.lambdaStopTestPrev;
      for (int i{0}; i < idxFiniteLB; i++) {
        (*b_workspace)[i] = b_TrialState.grad[i];
      }
      for (idx = 0; idx < iL0; idx++) {
        (*b_workspace)[(*finiteFixed)[idx] - 1] =
            (*b_workspace)[(*finiteFixed)[idx] - 1] + (*lambda)[idx];
      }
      for (idx = 0; idx < mLB; idx++) {
        (*b_workspace)[(*finiteLB)[idx] - 1] =
            (*b_workspace)[(*finiteLB)[idx] - 1] - (*lambda)[iL0 + idx];
      }
      if (mLB - 1 >= 0) {
        iL0 += mLB;
      }
      for (idx = 0; idx < mUB; idx++) {
        (*b_workspace)[(*finiteUB)[idx] - 1] =
            (*b_workspace)[(*finiteUB)[idx] - 1] + (*lambda)[iL0 + idx];
      }
      idxFiniteLB = WorkingSet.nVar;
      b_workspace = &memspace.workspace_double;
      s = 0.0;
      idx = 0;
      exitg1 = false;
      while ((!exitg1) && (idx <= idxFiniteLB - 1)) {
        dxTooSmall = ((!std::isinf((*b_workspace)[idx])) &&
                      (!std::isnan((*b_workspace)[idx])));
        if (!dxTooSmall) {
          exitg1 = true;
        } else {
          s = std::fmax(s, std::abs((*b_workspace)[idx]));
          idx++;
        }
      }
      nlpComplErrorTmp = stopping::computeComplError(
          b_TrialState.xstarsqp, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
          WorkingSet.indexUB, WorkingSet.sizes[4], ub,
          b_TrialState.lambdaStopTestPrev, WorkingSet.sizes[0] + 1);
      d = std::fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        MeritFunction.nlpDualFeasError = s;
        MeritFunction.nlpComplError = nlpComplErrorTmp;
        MeritFunction.firstOrderOpt = d;
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTest[k] = b_TrialState.lambdaStopTestPrev[k];
        }
      } else {
        for (k = 0; k <= mLambda; k++) {
          b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
        }
      }
    } else {
      for (k = 0; k <= mLambda; k++) {
        b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
      }
    }
    if (isFeasible &&
        (MeritFunction.nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction.nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags.done = true;
      b_TrialState.sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
        Flags.done = true;
        b_TrialState.sqpExitFlag = -3;
      } else {
        boolean_T guard1;
        guard1 = false;
        if (b_TrialState.sqpIterations > 0) {
          dxTooSmall = true;
          idx = 0;
          exitg1 = false;
          while ((!exitg1) && (idx <= nVar - 1)) {
            if (1.0E-6 * std::fmax(1.0, std::abs(b_TrialState.xstarsqp[idx])) <=
                std::abs(b_TrialState.delta_x[idx])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx++;
            }
          }
          if (dxTooSmall) {
            if (!isFeasible) {
              if (Flags.stepType != 2) {
                Flags.stepType = 2;
                Flags.failedLineSearch = false;
                Flags.stepAccepted = false;
                guard1 = true;
              } else {
                Flags.done = true;
                b_TrialState.sqpExitFlag = -2;
              }
            } else {
              nActiveConstr = WorkingSet.nActiveConstr - 1;
              if (WorkingSet.nActiveConstr > 0) {
                boolean_T guard2;
                for (k = 0; k <= nActiveConstr; k++) {
                  b_TrialState.lambda[k] = 0.0;
                }
                k = WorkingSet.nVar * WorkingSet.nActiveConstr;
                guard2 = false;
                if (k > 0) {
                  for (idx = 0; idx <= nActiveConstr; idx++) {
                    idxFiniteLB = WorkingSet.ldA * idx;
                    iL0 = b_QRManager.ldq * idx;
                    for (k = 0; k < nVar; k++) {
                      b_QRManager.QR[iL0 + k] =
                          WorkingSet.ATwset[idxFiniteLB + k];
                    }
                  }
                  guard2 = true;
                } else if (k == 0) {
                  b_QRManager.mrows = WorkingSet.nVar;
                  b_QRManager.ncols = WorkingSet.nActiveConstr;
                  b_QRManager.minRowCol = 0;
                } else {
                  guard2 = true;
                }
                if (guard2) {
                  b_QRManager.usedPivoting = true;
                  b_QRManager.mrows = WorkingSet.nVar;
                  b_QRManager.ncols = WorkingSet.nActiveConstr;
                  idxFiniteLB = WorkingSet.nVar;
                  iL0 = WorkingSet.nActiveConstr;
                  if (idxFiniteLB <= iL0) {
                    iL0 = idxFiniteLB;
                  }
                  b_QRManager.minRowCol = iL0;
                  ::coder::internal::lapack::xgeqp3(
                      b_QRManager.QR, WorkingSet.nVar, WorkingSet.nActiveConstr,
                      b_QRManager.jpvt, b_QRManager.tau);
                }
                QRManager::computeQ_(b_QRManager, b_QRManager.mrows);
                idxFiniteLB = WorkingSet.nVar;
                iL0 = WorkingSet.nActiveConstr;
                if (idxFiniteLB >= iL0) {
                  iL0 = idxFiniteLB;
                }
                smax = std::abs(b_QRManager.QR[0]) *
                       std::fmin(1.4901161193847656E-8,
                                 static_cast<double>(iL0) *
                                     2.2204460492503131E-16);
                idxFiniteLB = WorkingSet.nVar;
                mUB = WorkingSet.nActiveConstr;
                if (idxFiniteLB <= mUB) {
                  mUB = idxFiniteLB;
                }
                mLB = 0;
                iL0 = 0;
                while ((mLB < mUB) && (std::abs(b_QRManager.QR[iL0]) > smax)) {
                  mLB++;
                  iL0 = (iL0 + b_QRManager.ldq) + 1;
                }
                iL0 = WorkingSet.nVar;
                idxFiniteLB = WorkingSet.nVar;
                nActiveConstr = b_QRManager.ldq;
                b_workspace = &memspace.workspace_double;
                if ((iL0 != 0) && (idxFiniteLB != 0)) {
                  for (idx = 0; idx < idxFiniteLB; idx++) {
                    (*b_workspace)[idx] = 0.0;
                  }
                  idx = 0;
                  k = nActiveConstr * (idxFiniteLB - 1) + 1;
                  for (nVar = 1; nActiveConstr < 0 ? nVar >= k : nVar <= k;
                       nVar += nActiveConstr) {
                    smax = 0.0;
                    idxFiniteLB = (nVar + iL0) - 1;
                    for (int i{nVar}; i <= idxFiniteLB; i++) {
                      smax +=
                          b_QRManager.Q[i - 1] * b_TrialState.grad[i - nVar];
                    }
                    (*b_workspace)[idx] = (*b_workspace)[idx] + smax;
                    idx++;
                  }
                }
                if (mLB != 0) {
                  for (nActiveConstr = mLB; nActiveConstr >= 1;
                       nActiveConstr--) {
                    idxFiniteLB = (nActiveConstr +
                                   (nActiveConstr - 1) * b_QRManager.ldq) -
                                  1;
                    memspace.workspace_double[nActiveConstr - 1] =
                        memspace.workspace_double[nActiveConstr - 1] /
                        b_QRManager.QR[idxFiniteLB];
                    for (int i{0}; i <= nActiveConstr - 2; i++) {
                      iL0 = (nActiveConstr - i) - 2;
                      memspace.workspace_double[iL0] =
                          memspace.workspace_double[iL0] -
                          memspace.workspace_double[nActiveConstr - 1] *
                              b_QRManager.QR[(idxFiniteLB - i) - 1];
                    }
                  }
                }
                idxFiniteLB = WorkingSet.nActiveConstr;
                if (idxFiniteLB <= mUB) {
                  mUB = idxFiniteLB;
                }
                for (idx = 0; idx < mUB; idx++) {
                  b_TrialState.lambda[b_QRManager.jpvt[idx] - 1] =
                      memspace.workspace_double[idx];
                }
                k = WorkingSet.sizes[0] + 1;
                for (idx = k; idx <= mFixed; idx++) {
                  b_TrialState.lambda[idx - 1] = -b_TrialState.lambda[idx - 1];
                }
                qpactiveset::parseoutput::sortLambdaQP(
                    b_TrialState.lambda, WorkingSet.nActiveConstr,
                    WorkingSet.sizes, WorkingSet.isActiveIdx, WorkingSet.Wid,
                    WorkingSet.Wlocalidx, memspace.workspace_double);
                b_workspace = &memspace.workspace_double;
                nVar = WorkingSet.nVar;
                finiteFixed = &WorkingSet.indexFixed;
                mFixed = WorkingSet.sizes[0];
                finiteLB = &WorkingSet.indexLB;
                mLB = WorkingSet.sizes[3];
                finiteUB = &WorkingSet.indexUB;
                mUB = WorkingSet.sizes[4];
                lambda = &b_TrialState.lambda;
                for (int i{0}; i < nVar; i++) {
                  (*b_workspace)[i] = b_TrialState.grad[i];
                }
                for (idx = 0; idx < mFixed; idx++) {
                  (*b_workspace)[(*finiteFixed)[idx] - 1] =
                      (*b_workspace)[(*finiteFixed)[idx] - 1] + (*lambda)[idx];
                }
                for (idx = 0; idx < mLB; idx++) {
                  (*b_workspace)[(*finiteLB)[idx] - 1] =
                      (*b_workspace)[(*finiteLB)[idx] - 1] -
                      (*lambda)[mFixed + idx];
                }
                if (mLB - 1 < 0) {
                  iL0 = mFixed;
                } else {
                  iL0 = mFixed + mLB;
                }
                for (idx = 0; idx < mUB; idx++) {
                  (*b_workspace)[(*finiteUB)[idx] - 1] =
                      (*b_workspace)[(*finiteUB)[idx] - 1] +
                      (*lambda)[iL0 + idx];
                }
                nVar = WorkingSet.nVar;
                b_workspace = &memspace.workspace_double;
                smax = 0.0;
                idx = 0;
                exitg1 = false;
                while ((!exitg1) && (idx <= nVar - 1)) {
                  dxTooSmall = ((!std::isinf((*b_workspace)[idx])) &&
                                (!std::isnan((*b_workspace)[idx])));
                  if (!dxTooSmall) {
                    exitg1 = true;
                  } else {
                    smax = std::fmax(smax, std::abs((*b_workspace)[idx]));
                    idx++;
                  }
                }
                s = stopping::computeComplError(
                    b_TrialState.xstarsqp, WorkingSet.indexLB,
                    WorkingSet.sizes[3], lb, WorkingSet.indexUB,
                    WorkingSet.sizes[4], ub, b_TrialState.lambda,
                    WorkingSet.sizes[0] + 1);
                if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                    (s <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction.nlpDualFeasError = smax;
                  MeritFunction.nlpComplError = s;
                  MeritFunction.firstOrderOpt = std::fmax(smax, s);
                  for (k = 0; k <= mLambda; k++) {
                    b_TrialState.lambdaStopTest[k] = b_TrialState.lambda[k];
                  }
                  Flags.done = true;
                  b_TrialState.sqpExitFlag = 1;
                } else {
                  Flags.done = true;
                  b_TrialState.sqpExitFlag = 2;
                }
              } else {
                Flags.done = true;
                b_TrialState.sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (b_TrialState.sqpIterations >= 50) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          } else if (b_TrialState.FunctionEvaluations >=
                     runTimeOptions_MaxFunctionEvaluations) {
            Flags.done = true;
            b_TrialState.sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

boolean_T test_exit(struct_T &MeritFunction, const i_struct_T &WorkingSet,
                    h_struct_T &b_TrialState, const array<double, 1U> &lb,
                    const array<double, 1U> &ub,
                    int runTimeOptions_MaxFunctionEvaluations,
                    boolean_T &Flags_fevalOK, boolean_T &Flags_done,
                    boolean_T &Flags_stepAccepted,
                    boolean_T &Flags_failedLineSearch, int &Flags_stepType)
{
  const array<int, 1U> *finiteFixed;
  const array<int, 1U> *finiteLB;
  const array<int, 1U> *finiteUB;
  array<double, 1U> *lambda;
  array<double, 1U> *workspace;
  double s;
  double smax;
  int b_mLB;
  int b_mUB;
  int i;
  int iL0;
  int k;
  int mLB;
  int mLambda;
  int mUB;
  int nVar;
  boolean_T Flags_gradOK;
  boolean_T exitg1;
  boolean_T isFeasible;
  Flags_fevalOK = true;
  Flags_done = false;
  Flags_stepAccepted = false;
  Flags_failedLineSearch = false;
  Flags_stepType = 1;
  nVar = WorkingSet.nVar;
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mLambda =
      ((WorkingSet.sizes[0] + WorkingSet.sizes[3]) + WorkingSet.sizes[4]) - 1;
  for (k = 0; k <= mLambda; k++) {
    b_TrialState.lambdaStopTest[k] = b_TrialState.lambdasqp[k];
  }
  workspace = &b_TrialState.gradLag;
  iL0 = WorkingSet.nVar;
  finiteFixed = &WorkingSet.indexFixed;
  k = WorkingSet.sizes[0];
  finiteLB = &WorkingSet.indexLB;
  b_mLB = WorkingSet.sizes[3];
  finiteUB = &WorkingSet.indexUB;
  b_mUB = WorkingSet.sizes[4];
  lambda = &b_TrialState.lambdaStopTest;
  for (i = 0; i < iL0; i++) {
    (*workspace)[i] = b_TrialState.grad[i];
  }
  for (i = 0; i < k; i++) {
    (*workspace)[(*finiteFixed)[i] - 1] =
        (*workspace)[(*finiteFixed)[i] - 1] + (*lambda)[i];
  }
  for (i = 0; i < b_mLB; i++) {
    (*workspace)[(*finiteLB)[i] - 1] =
        (*workspace)[(*finiteLB)[i] - 1] - (*lambda)[k + i];
  }
  if (b_mLB - 1 < 0) {
    iL0 = k;
  } else {
    iL0 = k + b_mLB;
  }
  for (i = 0; i < b_mUB; i++) {
    (*workspace)[(*finiteUB)[i] - 1] =
        (*workspace)[(*finiteUB)[i] - 1] + (*lambda)[iL0 + i];
  }
  if (WorkingSet.nVar < 1) {
    iL0 = 0;
  } else {
    iL0 = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(b_TrialState.grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = std::abs(b_TrialState.grad[k - 1]);
        if (s > smax) {
          iL0 = k;
          smax = s;
        }
      }
    }
  }
  s = std::fmax(1.0, std::abs(b_TrialState.grad[iL0 - 1]));
  if (std::isinf(s)) {
    s = 1.0;
  }
  smax = 0.0;
  for (i = 0; i < mLB; i++) {
    iL0 = WorkingSet.indexLB[i] - 1;
    smax = std::fmax(smax, lb[iL0] - b_TrialState.xstarsqp[iL0]);
  }
  for (i = 0; i < mUB; i++) {
    iL0 = WorkingSet.indexUB[i] - 1;
    smax = std::fmax(smax, b_TrialState.xstarsqp[iL0] - ub[iL0]);
  }
  MeritFunction.nlpPrimalFeasError = smax;
  MeritFunction.feasRelativeFactor = std::fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction.feasRelativeFactor);
  nVar = WorkingSet.nVar;
  workspace = &b_TrialState.gradLag;
  Flags_gradOK = true;
  smax = 0.0;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i <= nVar - 1)) {
    Flags_gradOK =
        ((!std::isinf((*workspace)[i])) && (!std::isnan((*workspace)[i])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs((*workspace)[i]));
      i++;
    }
  }
  MeritFunction.nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    Flags_done = true;
    if (isFeasible) {
      b_TrialState.sqpExitFlag = 2;
    } else {
      b_TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt =
        std::fmax(MeritFunction.nlpDualFeasError, 0.0);
    for (k = 0; k <= mLambda; k++) {
      b_TrialState.lambdaStopTestPrev[k] = b_TrialState.lambdaStopTest[k];
    }
    if (isFeasible && (MeritFunction.nlpDualFeasError <= 1.0E-6 * s)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = 1;
    } else if (isFeasible && (b_TrialState.sqpFval < -1.0E+20)) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = -3;
    } else if (b_TrialState.FunctionEvaluations >=
               runTimeOptions_MaxFunctionEvaluations) {
      Flags_done = true;
      b_TrialState.sqpExitFlag = 0;
    }
  }
  return Flags_gradOK;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (test_exit.cpp)
