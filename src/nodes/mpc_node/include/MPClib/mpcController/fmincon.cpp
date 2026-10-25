//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fmincon.cpp
//
// Code generation for function 'fmincon'
//

// Include files
#include "fmincon.h"
#include "anonymous_function.h"
#include "compressBounds.h"
#include "computeFiniteDifferences.h"
#include "driver.h"
#include "factoryConstruct.h"
#include "factoryConstruct1.h"
#include "mpcController.h"
#include "mpcController_internal_types.h"
#include "mpcController_internal_types1.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "stickyStruct.h"
#include "coder_array.h"
#include "coder_bounded_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
void fmincon(const anonymous_function &fun, array<double, 1U> &x0,
             const array<double, 1U> &lb, const array<double, 1U> &ub)
{
  internal::i_stickyStruct FcnEvaluator;
  array<double, 2U> Hessian;
  d_struct_T QRManager;
  e_struct_T CholManager;
  f_struct_T QPObjective;
  g_struct_T memspace;
  h_struct_T TrialState;
  i_struct_T WorkingSet;
  k_struct_T FiniteDifferences;
  struct_T MeritFunction;
  double exitflag;
  int i;
  int idxFillStart;
  int mConstrMax;
  int mFixed;
  int maxDims;
  int nVar;
  nVar = x0.size(0) - 1;
  exitflag = rtInf;
  if ((lb.size(0) != 0) && (ub.size(0) != 0)) {
    boolean_T exitg1;
    idxFillStart = 0;
    exitg1 = false;
    while ((!exitg1) && (idxFillStart <= x0.size(0) - 1)) {
      if (lb[idxFillStart] > ub[idxFillStart]) {
        exitflag = -2.0;
        exitg1 = true;
      } else {
        idxFillStart++;
      }
    }
  }
  mConstrMax = (lb.size(0) + ub.size(0)) + 1;
  maxDims = x0.size(0) + 1;
  if (maxDims < mConstrMax) {
    maxDims = mConstrMax;
  }
  Hessian.set_size(x0.size(0), x0.size(0));
  idxFillStart = x0.size(0) * x0.size(0);
  for (i = 0; i < idxFillStart; i++) {
    Hessian[i] = 0.0;
  }
  for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
    Hessian[idxFillStart + Hessian.size(0) * idxFillStart] = 1.0;
  }
  if (!(exitflag == -2.0)) {
    boolean_T hasLB;
    boolean_T hasUB;
    optim::coder::fminconsqp::TrialState::factoryConstruct(
        x0.size(0) + 1, mConstrMax, x0, TrialState);
    for (idxFillStart = 0; idxFillStart <= nVar; idxFillStart++) {
      TrialState.xstarsqp[idxFillStart] = x0[idxFillStart];
    }
    FcnEvaluator.next.next.next.next.next.next.next.next.value = fun;
    optim::coder::utils::FiniteDifferences::factoryConstruct(
        fun, x0.size(0), lb, ub, FiniteDifferences);
    QRManager.ldq = maxDims;
    QRManager.QR.set_size(maxDims, maxDims);
    QRManager.Q.set_size(maxDims, maxDims);
    idxFillStart = maxDims * maxDims;
    for (i = 0; i < idxFillStart; i++) {
      QRManager.Q[i] = 0.0;
    }
    QRManager.jpvt.set_size(maxDims);
    for (i = 0; i < maxDims; i++) {
      QRManager.jpvt[i] = 0;
    }
    QRManager.mrows = 0;
    QRManager.ncols = 0;
    QRManager.tau.set_size(maxDims);
    QRManager.minRowCol = 0;
    QRManager.usedPivoting = false;
    CholManager.FMat.set_size(maxDims, maxDims);
    CholManager.ldm = maxDims;
    CholManager.ndims = 0;
    CholManager.info = 0;
    CholManager.scaleFactor = 0.0;
    CholManager.ConvexCheck = true;
    CholManager.regTol_ = rtInf;
    CholManager.workspace_ = rtInf;
    CholManager.workspace2_ = rtInf;
    QPObjective.grad.set_size(x0.size(0) + 1);
    QPObjective.Hx.set_size(x0.size(0));
    QPObjective.maxVar = x0.size(0) + 1;
    QPObjective.beta = 0.0;
    QPObjective.rho = 0.0;
    QPObjective.prev_objtype = 3;
    QPObjective.prev_nvar = 0;
    QPObjective.prev_hasLinear = false;
    QPObjective.gammaScalar = 0.0;
    QPObjective.nvar = x0.size(0);
    QPObjective.hasLinear = true;
    QPObjective.objtype = 3;
    idxFillStart = x0.size(0) + 1;
    if (idxFillStart < 2) {
      idxFillStart = 2;
    }
    memspace.workspace_double.set_size(maxDims, idxFillStart);
    memspace.workspace_int.set_size(maxDims);
    memspace.workspace_sort.set_size(maxDims);
    nVar = x0.size(0);
    idxFillStart = x0.size(0) + 1;
    WorkingSet.mConstr = 0;
    WorkingSet.mConstrOrig = 0;
    WorkingSet.mConstrMax = mConstrMax;
    WorkingSet.nVar = nVar;
    WorkingSet.nVarOrig = nVar;
    WorkingSet.nVarMax = idxFillStart;
    WorkingSet.ldA = idxFillStart;
    WorkingSet.Aineq.size[0] = 0;
    WorkingSet.Aeq.size[0] = 0;
    WorkingSet.lb.set_size(idxFillStart);
    WorkingSet.ub.set_size(idxFillStart);
    WorkingSet.indexLB.set_size(idxFillStart);
    WorkingSet.indexUB.set_size(idxFillStart);
    WorkingSet.indexFixed.set_size(idxFillStart);
    WorkingSet.mEqRemoved = 0;
    WorkingSet.ATwset.set_size(idxFillStart * mConstrMax);
    WorkingSet.bwset.set_size(mConstrMax);
    WorkingSet.nActiveConstr = 0;
    WorkingSet.maxConstrWorkspace.set_size(mConstrMax);
    for (int b_i{0}; b_i < 5; b_i++) {
      WorkingSet.sizes[b_i] = 0;
      WorkingSet.sizesNormal[b_i] = 0;
      WorkingSet.sizesPhaseOne[b_i] = 0;
      WorkingSet.sizesRegularized[b_i] = 0;
      WorkingSet.sizesRegPhaseOne[b_i] = 0;
    }
    for (int b_i{0}; b_i < 6; b_i++) {
      WorkingSet.isActiveIdx[b_i] = 0;
      WorkingSet.isActiveIdxNormal[b_i] = 0;
      WorkingSet.isActiveIdxPhaseOne[b_i] = 0;
      WorkingSet.isActiveIdxRegularized[b_i] = 0;
      WorkingSet.isActiveIdxRegPhaseOne[b_i] = 0;
    }
    WorkingSet.isActiveConstr.set_size(mConstrMax);
    WorkingSet.Wid.set_size(mConstrMax);
    WorkingSet.Wlocalidx.set_size(mConstrMax);
    for (int b_i{0}; b_i < 5; b_i++) {
      WorkingSet.nWConstr[b_i] = 0;
    }
    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    nVar = optim::coder::qpactiveset::initialize::compressBounds(
        x0.size(0), WorkingSet.indexLB, WorkingSet.indexUB,
        WorkingSet.indexFixed, lb, ub, maxDims, mFixed);
    WorkingSet.mConstrMax = mConstrMax;
    idxFillStart = (nVar + maxDims) + mFixed;
    WorkingSet.mConstr = idxFillStart;
    WorkingSet.mConstrOrig = idxFillStart;
    WorkingSet.sizes[0] = mFixed;
    WorkingSet.sizes[1] = 0;
    WorkingSet.sizes[2] = 0;
    WorkingSet.sizes[3] = nVar;
    WorkingSet.sizes[4] = maxDims;
    WorkingSet.sizesPhaseOne[0] = mFixed;
    WorkingSet.sizesPhaseOne[1] = 0;
    WorkingSet.sizesPhaseOne[2] = 0;
    WorkingSet.sizesPhaseOne[3] = nVar + 1;
    WorkingSet.sizesPhaseOne[4] = maxDims;
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = nVar;
    WorkingSet.isActiveIdx[5] = maxDims;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.sizesNormal[idxFillStart] = WorkingSet.sizes[idxFillStart];
      WorkingSet.sizesRegularized[idxFillStart] =
          WorkingSet.sizes[idxFillStart];
      WorkingSet.sizesRegPhaseOne[idxFillStart] =
          WorkingSet.sizesPhaseOne[idxFillStart];
      WorkingSet.isActiveIdx[idxFillStart + 1] +=
          WorkingSet.isActiveIdx[idxFillStart];
    }
    for (i = 0; i < 6; i++) {
      WorkingSet.isActiveIdxNormal[i] = WorkingSet.isActiveIdx[i];
    }
    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = nVar + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = maxDims;
    for (idxFillStart = 0; idxFillStart < 5; idxFillStart++) {
      WorkingSet.isActiveIdxPhaseOne[idxFillStart + 1] +=
          WorkingSet.isActiveIdxPhaseOne[idxFillStart];
    }
    for (i = 0; i < 6; i++) {
      WorkingSet.isActiveIdxRegularized[i] = WorkingSet.isActiveIdx[i];
      WorkingSet.isActiveIdxRegPhaseOne[i] = WorkingSet.isActiveIdxPhaseOne[i];
    }
    if (lb.size(0) != 0) {
      for (int b_i{0}; b_i < nVar; b_i++) {
        TrialState.xstarsqp[WorkingSet.indexLB[b_i] - 1] =
            std::fmax(TrialState.xstarsqp[WorkingSet.indexLB[b_i] - 1],
                      lb[WorkingSet.indexLB[b_i] - 1]);
      }
    }
    if (ub.size(0) != 0) {
      for (int b_i{0}; b_i < maxDims; b_i++) {
        TrialState.xstarsqp[WorkingSet.indexUB[b_i] - 1] =
            std::fmin(TrialState.xstarsqp[WorkingSet.indexUB[b_i] - 1],
                      ub[WorkingSet.indexUB[b_i] - 1]);
      }
      for (int b_i{0}; b_i < mFixed; b_i++) {
        TrialState.xstarsqp[WorkingSet.indexFixed[b_i] - 1] =
            ub[WorkingSet.indexFixed[b_i] - 1];
      }
    }
    exitflag = mpcController_anonFcn1(
        fun.workspace.x_current, fun.workspace.refHorizon,
        fun.workspace.u_last_applied, fun.workspace.Np, fun.workspace.Ts,
        fun.workspace.L, fun.workspace.Q, fun.workspace.R, fun.workspace.S,
        TrialState.xstarsqp);
    TrialState.sqpFval = exitflag;
    optim::coder::utils::FiniteDifferences::computeFiniteDifferences(
        FiniteDifferences, exitflag, TrialState.xstarsqp, TrialState.grad, lb,
        ub);
    TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
    hasLB = (lb.size(0) != 0);
    hasUB = (ub.size(0) != 0);
    if (hasLB) {
      for (int b_i{0}; b_i < nVar; b_i++) {
        WorkingSet.lb[WorkingSet.indexLB[b_i] - 1] =
            -lb[WorkingSet.indexLB[b_i] - 1] + x0[WorkingSet.indexLB[b_i] - 1];
      }
    }
    if (hasUB) {
      for (int b_i{0}; b_i < maxDims; b_i++) {
        WorkingSet.ub[WorkingSet.indexUB[b_i] - 1] =
            ub[WorkingSet.indexUB[b_i] - 1] - x0[WorkingSet.indexUB[b_i] - 1];
      }
    }
    if (hasLB && hasUB) {
      for (int b_i{0}; b_i < mFixed; b_i++) {
        double d;
        d = ub[WorkingSet.indexFixed[b_i] - 1] -
            x0[WorkingSet.indexFixed[b_i] - 1];
        WorkingSet.ub[WorkingSet.indexFixed[b_i] - 1] = d;
        WorkingSet.bwset[b_i] = d;
      }
    }
    optim::coder::qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
    idxFillStart = WorkingSet.isActiveIdx[2];
    i = WorkingSet.mConstrMax;
    for (int b_i{idxFillStart}; b_i <= i; b_i++) {
      WorkingSet.isActiveConstr[b_i - 1] = false;
    }
    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    idxFillStart = WorkingSet.sizes[0];
    for (mFixed = 0; mFixed < idxFillStart; mFixed++) {
      WorkingSet.Wid[mFixed] = 1;
      WorkingSet.Wlocalidx[mFixed] = mFixed + 1;
      WorkingSet.isActiveConstr[mFixed] = true;
      nVar = WorkingSet.ldA * mFixed;
      i = WorkingSet.indexFixed[mFixed];
      for (int b_i{0}; b_i <= i - 2; b_i++) {
        WorkingSet.ATwset[b_i + nVar] = 0.0;
      }
      WorkingSet.ATwset[(WorkingSet.indexFixed[mFixed] + nVar) - 1] = 1.0;
      i = WorkingSet.indexFixed[mFixed] + 1;
      maxDims = WorkingSet.nVar;
      for (int b_i{i}; b_i <= maxDims; b_i++) {
        WorkingSet.ATwset[(b_i + nVar) - 1] = 0.0;
      }
      WorkingSet.bwset[mFixed] =
          WorkingSet.ub[WorkingSet.indexFixed[mFixed] - 1];
    }
    MeritFunction.initFval = exitflag;
    MeritFunction.penaltyParam = 1.0;
    MeritFunction.threshold = 0.0001;
    MeritFunction.nPenaltyDecreases = 0;
    MeritFunction.linearizedConstrViol = 0.0;
    MeritFunction.initConstrViolationEq = 0.0;
    MeritFunction.initConstrViolationIneq = 0.0;
    MeritFunction.phi = 0.0;
    MeritFunction.phiPrimePlus = 0.0;
    MeritFunction.phiFullStep = 0.0;
    MeritFunction.feasRelativeFactor = 0.0;
    MeritFunction.nlpPrimalFeasError = 0.0;
    MeritFunction.nlpDualFeasError = 0.0;
    MeritFunction.nlpComplError = 0.0;
    MeritFunction.firstOrderOpt = 0.0;
    MeritFunction.hasObjective = true;
    optim::coder::fminconsqp::driver(Hessian, lb, ub, TrialState, MeritFunction,
                                     FcnEvaluator, FiniteDifferences, memspace,
                                     WorkingSet, QRManager, CholManager,
                                     QPObjective, 100 * x0.size(0));
    x0.set_size(TrialState.xstarsqp.size(0));
    idxFillStart = TrialState.xstarsqp.size(0);
    for (i = 0; i < idxFillStart; i++) {
      x0[i] = TrialState.xstarsqp[i];
    }
  }
}

} // namespace coder

// End of code generation (fmincon.cpp)
