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
#include "computeGradLag.h"
#include "computeQ_.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xgemv.h"
#include "xgeqp3.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void b_test_exit(j_struct_T &Flags, c_struct_T &memspace,
                 b_struct_T &MeritFunction, const h_struct_T &WorkingSet,
                 g_struct_T &TrialState, d_struct_T &b_QRManager,
                 const double lb[20], const double ub[20])
{
  double optimRelativeFactor;
  double s;
  double smax;
  int i;
  int i1;
  int idx_max;
  int mFixed;
  int nVar;
  boolean_T dxTooSmall;
  boolean_T exitg1;
  boolean_T isFeasible;
  nVar = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  i = static_cast<unsigned char>((WorkingSet.sizes[0] + WorkingSet.sizes[3]) +
                                 WorkingSet.sizes[4]);
  if (i - 1 >= 0) {
    std::copy(&TrialState.lambdasqp[0], &TrialState.lambdasqp[i],
              &TrialState.lambdaStopTest[0]);
  }
  stopping::computeGradLag(TrialState.gradLag, WorkingSet.nVar, TrialState.grad,
                           WorkingSet.indexFixed, WorkingSet.sizes[0],
                           WorkingSet.indexLB, WorkingSet.sizes[3],
                           WorkingSet.indexUB, WorkingSet.sizes[4],
                           TrialState.lambdaStopTest);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(TrialState.grad[0]);
      for (int k{2}; k <= nVar; k++) {
        s = std::abs(TrialState.grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  optimRelativeFactor = std::fmax(1.0, std::abs(TrialState.grad[idx_max - 1]));
  if (std::isinf(optimRelativeFactor)) {
    optimRelativeFactor = 1.0;
  }
  smax = 0.0;
  i1 = static_cast<unsigned char>(WorkingSet.sizes[3]);
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet.indexLB[idx_max] - 1;
    smax = std::fmax(smax, lb[nVar] - TrialState.xstarsqp[nVar]);
  }
  i1 = static_cast<unsigned char>(WorkingSet.sizes[4]);
  for (idx_max = 0; idx_max < i1; idx_max++) {
    nVar = WorkingSet.indexUB[idx_max] - 1;
    smax = std::fmax(smax, TrialState.xstarsqp[nVar] - ub[nVar]);
  }
  MeritFunction.nlpPrimalFeasError = smax;
  if (TrialState.sqpIterations == 0) {
    MeritFunction.feasRelativeFactor = std::fmax(1.0, smax);
  }
  isFeasible = (smax <= 1.0E-6 * MeritFunction.feasRelativeFactor);
  dxTooSmall = true;
  smax = 0.0;
  i1 = static_cast<unsigned char>(WorkingSet.nVar);
  idx_max = 0;
  exitg1 = false;
  while ((!exitg1) && (idx_max <= i1 - 1)) {
    dxTooSmall = ((!std::isinf(TrialState.gradLag[idx_max])) &&
                  (!std::isnan(TrialState.gradLag[idx_max])));
    if (!dxTooSmall) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs(TrialState.gradLag[idx_max]));
      idx_max++;
    }
  }
  MeritFunction.nlpDualFeasError = smax;
  if (!dxTooSmall) {
    Flags.done = true;
    if (isFeasible) {
      TrialState.sqpExitFlag = 2;
    } else {
      TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = stopping::computeComplError(
        TrialState.xstarsqp, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
        WorkingSet.indexUB, WorkingSet.sizes[4], ub, TrialState.lambdaStopTest,
        WorkingSet.sizes[0] + 1);
    smax = std::fmax(smax, MeritFunction.nlpComplError);
    MeritFunction.firstOrderOpt = smax;
    if (TrialState.sqpIterations > 1) {
      double d;
      double nlpComplErrorTmp;
      stopping::b_computeGradLag(
          memspace.workspace_double, WorkingSet.nVar, TrialState.grad,
          WorkingSet.indexFixed, WorkingSet.sizes[0], WorkingSet.indexLB,
          WorkingSet.sizes[3], WorkingSet.indexUB, WorkingSet.sizes[4],
          TrialState.lambdaStopTestPrev);
      s = 0.0;
      idx_max = 0;
      while ((idx_max <= i1 - 1) &&
             ((!std::isinf(memspace.workspace_double[idx_max])) &&
              (!std::isnan(memspace.workspace_double[idx_max])))) {
        s = std::fmax(s, std::abs(memspace.workspace_double[idx_max]));
        idx_max++;
      }
      nlpComplErrorTmp = stopping::computeComplError(
          TrialState.xstarsqp, WorkingSet.indexLB, WorkingSet.sizes[3], lb,
          WorkingSet.indexUB, WorkingSet.sizes[4], ub,
          TrialState.lambdaStopTestPrev, WorkingSet.sizes[0] + 1);
      d = std::fmax(s, nlpComplErrorTmp);
      if (d < smax) {
        MeritFunction.nlpDualFeasError = s;
        MeritFunction.nlpComplError = nlpComplErrorTmp;
        MeritFunction.firstOrderOpt = d;
        if (i - 1 >= 0) {
          std::copy(&TrialState.lambdaStopTestPrev[0],
                    &TrialState.lambdaStopTestPrev[i],
                    &TrialState.lambdaStopTest[0]);
        }
      } else if (i - 1 >= 0) {
        std::copy(&TrialState.lambdaStopTest[0], &TrialState.lambdaStopTest[i],
                  &TrialState.lambdaStopTestPrev[0]);
      }
    } else if (i - 1 >= 0) {
      std::copy(&TrialState.lambdaStopTest[0], &TrialState.lambdaStopTest[i],
                &TrialState.lambdaStopTestPrev[0]);
    }
    if (isFeasible &&
        (MeritFunction.nlpDualFeasError <= 1.0E-6 * optimRelativeFactor) &&
        (MeritFunction.nlpComplError <= 1.0E-6 * optimRelativeFactor)) {
      Flags.done = true;
      TrialState.sqpExitFlag = 1;
    } else {
      Flags.done = false;
      if (isFeasible && (TrialState.sqpFval < -1.0E+20)) {
        Flags.done = true;
        TrialState.sqpExitFlag = -3;
      } else {
        boolean_T guard1;
        guard1 = false;
        if (TrialState.sqpIterations > 0) {
          dxTooSmall = true;
          idx_max = 0;
          exitg1 = false;
          while ((!exitg1) && (idx_max <= i1 - 1)) {
            if (1.0E-6 *
                    std::fmax(1.0, std::abs(TrialState.xstarsqp[idx_max])) <=
                std::abs(TrialState.delta_x[idx_max])) {
              dxTooSmall = false;
              exitg1 = true;
            } else {
              idx_max++;
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
                TrialState.sqpExitFlag = -2;
              }
            } else {
              nVar = WorkingSet.nActiveConstr - 1;
              if (WorkingSet.nActiveConstr > 0) {
                int b_k;
                int rankR;
                for (int k{0}; k <= nVar; k++) {
                  TrialState.lambda[k] = 0.0;
                  idx_max = 21 * k;
                  rankR = 41 * k;
                  for (b_k = 0; b_k < i1; b_k++) {
                    b_QRManager.QR[rankR + b_k] =
                        WorkingSet.ATwset[idx_max + b_k];
                  }
                }
                b_QRManager.usedPivoting = true;
                b_QRManager.mrows = WorkingSet.nVar;
                b_QRManager.ncols = WorkingSet.nActiveConstr;
                nVar = WorkingSet.nVar;
                b_k = WorkingSet.nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }
                b_QRManager.minRowCol = b_k;
                ::coder::internal::lapack::xgeqp3(
                    b_QRManager.QR, WorkingSet.nVar, WorkingSet.nActiveConstr,
                    b_QRManager.jpvt, b_QRManager.tau);
                QRManager::computeQ_(b_QRManager, WorkingSet.nVar);
                nVar = WorkingSet.nVar;
                idx_max = WorkingSet.nActiveConstr;
                if (nVar >= idx_max) {
                  idx_max = nVar;
                }
                smax = std::abs(b_QRManager.QR[0]) *
                       std::fmin(1.4901161193847656E-8,
                                 static_cast<double>(idx_max) *
                                     2.2204460492503131E-16);
                rankR = 0;
                nVar = 0;
                while ((rankR < b_k) &&
                       (std::abs(b_QRManager.QR[nVar]) > smax)) {
                  rankR++;
                  nVar += 42;
                }
                ::coder::internal::blas::xgemv(WorkingSet.nVar, WorkingSet.nVar,
                                               b_QRManager.Q, TrialState.grad,
                                               memspace.workspace_double);
                if (rankR != 0) {
                  for (int k{rankR}; k >= 1; k--) {
                    nVar = (k + (k - 1) * 41) - 1;
                    memspace.workspace_double[k - 1] /= b_QRManager.QR[nVar];
                    for (int b_i{0}; b_i <= k - 2; b_i++) {
                      idx_max = (k - b_i) - 2;
                      memspace.workspace_double[idx_max] -=
                          memspace.workspace_double[k - 1] *
                          b_QRManager.QR[(nVar - b_i) - 1];
                    }
                  }
                }
                nVar = WorkingSet.nActiveConstr;
                if (nVar <= b_k) {
                  b_k = nVar;
                }
                nVar = static_cast<unsigned char>(b_k);
                for (idx_max = 0; idx_max < nVar; idx_max++) {
                  TrialState.lambda[b_QRManager.jpvt[idx_max] - 1] =
                      memspace.workspace_double[idx_max];
                }
                nVar = WorkingSet.sizes[0] + 1;
                for (idx_max = nVar; idx_max <= mFixed; idx_max++) {
                  TrialState.lambda[idx_max - 1] =
                      -TrialState.lambda[idx_max - 1];
                }
                qpactiveset::parseoutput::sortLambdaQP(
                    TrialState.lambda, WorkingSet.nActiveConstr,
                    WorkingSet.sizes, WorkingSet.isActiveIdx, WorkingSet.Wid,
                    WorkingSet.Wlocalidx, memspace.workspace_double);
                stopping::b_computeGradLag(
                    memspace.workspace_double, WorkingSet.nVar, TrialState.grad,
                    WorkingSet.indexFixed, WorkingSet.sizes[0],
                    WorkingSet.indexLB, WorkingSet.sizes[3], WorkingSet.indexUB,
                    WorkingSet.sizes[4], TrialState.lambda);
                smax = 0.0;
                idx_max = 0;
                while ((idx_max <= i1 - 1) &&
                       ((!std::isinf(memspace.workspace_double[idx_max])) &&
                        (!std::isnan(memspace.workspace_double[idx_max])))) {
                  smax = std::fmax(
                      smax, std::abs(memspace.workspace_double[idx_max]));
                  idx_max++;
                }
                s = stopping::computeComplError(
                    TrialState.xstarsqp, WorkingSet.indexLB,
                    WorkingSet.sizes[3], lb, WorkingSet.indexUB,
                    WorkingSet.sizes[4], ub, TrialState.lambda,
                    WorkingSet.sizes[0] + 1);
                if ((smax <= 1.0E-6 * optimRelativeFactor) &&
                    (s <= 1.0E-6 * optimRelativeFactor)) {
                  MeritFunction.nlpDualFeasError = smax;
                  MeritFunction.nlpComplError = s;
                  MeritFunction.firstOrderOpt = std::fmax(smax, s);
                  if (i - 1 >= 0) {
                    std::copy(&TrialState.lambda[0], &TrialState.lambda[i],
                              &TrialState.lambdaStopTest[0]);
                  }
                  Flags.done = true;
                  TrialState.sqpExitFlag = 1;
                } else {
                  Flags.done = true;
                  TrialState.sqpExitFlag = 2;
                }
              } else {
                Flags.done = true;
                TrialState.sqpExitFlag = 2;
              }
            }
          } else {
            guard1 = true;
          }
        } else {
          guard1 = true;
        }
        if (guard1) {
          if (TrialState.sqpIterations >= 50) {
            Flags.done = true;
            TrialState.sqpExitFlag = 0;
          } else if (TrialState.FunctionEvaluations >= 2000) {
            Flags.done = true;
            TrialState.sqpExitFlag = 0;
          }
        }
      }
    }
  }
}

boolean_T test_exit(b_struct_T &MeritFunction, const h_struct_T &WorkingSet,
                    g_struct_T &TrialState, const double lb[20],
                    const double ub[20], boolean_T &Flags_fevalOK,
                    boolean_T &Flags_done, boolean_T &Flags_stepAccepted,
                    boolean_T &Flags_failedLineSearch, int &Flags_stepType)
{
  double s;
  double smax;
  int i;
  int idx_max;
  int k;
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
  i = static_cast<unsigned char>((WorkingSet.sizes[0] + WorkingSet.sizes[3]) +
                                 WorkingSet.sizes[4]);
  if (i - 1 >= 0) {
    std::copy(&TrialState.lambdasqp[0], &TrialState.lambdasqp[i],
              &TrialState.lambdaStopTest[0]);
  }
  stopping::computeGradLag(TrialState.gradLag, WorkingSet.nVar, TrialState.grad,
                           WorkingSet.indexFixed, WorkingSet.sizes[0],
                           WorkingSet.indexLB, WorkingSet.sizes[3],
                           WorkingSet.indexUB, WorkingSet.sizes[4],
                           TrialState.lambdaStopTest);
  if (WorkingSet.nVar < 1) {
    idx_max = 0;
  } else {
    idx_max = 1;
    if (WorkingSet.nVar > 1) {
      smax = std::abs(TrialState.grad[0]);
      for (k = 2; k <= nVar; k++) {
        s = std::abs(TrialState.grad[k - 1]);
        if (s > smax) {
          idx_max = k;
          smax = s;
        }
      }
    }
  }
  s = std::fmax(1.0, std::abs(TrialState.grad[idx_max - 1]));
  if (std::isinf(s)) {
    s = 1.0;
  }
  smax = 0.0;
  idx_max = static_cast<unsigned char>(WorkingSet.sizes[3]);
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet.indexLB[k] - 1;
    smax = std::fmax(smax, lb[nVar] - TrialState.xstarsqp[nVar]);
  }
  idx_max = static_cast<unsigned char>(WorkingSet.sizes[4]);
  for (k = 0; k < idx_max; k++) {
    nVar = WorkingSet.indexUB[k] - 1;
    smax = std::fmax(smax, TrialState.xstarsqp[nVar] - ub[nVar]);
  }
  MeritFunction.nlpPrimalFeasError = smax;
  MeritFunction.feasRelativeFactor = std::fmax(1.0, smax);
  isFeasible = (smax <= 1.0E-6 * MeritFunction.feasRelativeFactor);
  Flags_gradOK = true;
  smax = 0.0;
  idx_max = static_cast<unsigned char>(WorkingSet.nVar);
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k <= idx_max - 1)) {
    Flags_gradOK = ((!std::isinf(TrialState.gradLag[k])) &&
                    (!std::isnan(TrialState.gradLag[k])));
    if (!Flags_gradOK) {
      exitg1 = true;
    } else {
      smax = std::fmax(smax, std::abs(TrialState.gradLag[k]));
      k++;
    }
  }
  MeritFunction.nlpDualFeasError = smax;
  if (!Flags_gradOK) {
    Flags_done = true;
    if (isFeasible) {
      TrialState.sqpExitFlag = 2;
    } else {
      TrialState.sqpExitFlag = -2;
    }
  } else {
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt = smax;
    if (i - 1 >= 0) {
      std::copy(&TrialState.lambdaStopTest[0], &TrialState.lambdaStopTest[i],
                &TrialState.lambdaStopTestPrev[0]);
    }
    if (isFeasible && (smax <= 1.0E-6 * s)) {
      Flags_done = true;
      TrialState.sqpExitFlag = 1;
    } else if (isFeasible && (TrialState.sqpFval < -1.0E+20)) {
      Flags_done = true;
      TrialState.sqpExitFlag = -3;
    } else if (TrialState.FunctionEvaluations >= 2000) {
      Flags_done = true;
      TrialState.sqpExitFlag = 0;
    }
  }
  return Flags_gradOK;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (test_exit.cpp)
