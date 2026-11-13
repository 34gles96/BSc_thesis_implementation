//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController.cpp
//
// Code generation for function 'mpcController'
//

// Include files
#include "mpcController.h"
#include "anonymous_function.h"
#include "computeForwardDifferences.h"
#include "driver1.h"
#include "factoryConstruct.h"
#include "mpcController_internal_types.h"
#include "mpcController_internal_types1.h"
#include "mpcController_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "stickyStruct.h"
#include "rt_defines.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

void mpcController(const double x_current[4], const double refHorizon[40],
                   const double u_last_applied[2], const struct0_T *params,
                   double u_optimal[2], double U_optimal[20])
{
  coder::internal::i_stickyStruct FcnEvaluator;
  b_struct_T MeritFunction;
  c_struct_T memspace;
  d_struct_T QRManager;
  e_struct_T CholManager;
  g_struct_T TrialState;
  h_struct_T WorkingSet;
  k_struct_T FiniteDifferences;
  struct_T QPObjective;
  double lowerBound[20];
  double upperBound[20];
  double d;
  double exitflag;
  int colOffsetATw;
  int i;
  signed char b_obj_tmp[5];
  signed char obj_tmp[5];
  boolean_T exitg1;
  //  Fixed prediction horizon for generated C++ code
  //  Validate the runtime configuration
  //  Validate the reference matrix
  //  MPC parameters
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.Ts =
      params->Ts;
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.L =
      params->L;
  std::copy(&params->Q[0], &params->Q[16],
            &FcnEvaluator.next.next.next.next.next.next.next.next.value
                 .workspace.Q[0]);
  //  State tracking error
  //  Control effort
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.R[0] =
      params->R[0];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.S[0] =
      params->S[0];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.R[1] =
      params->R[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.S[1] =
      params->S[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.R[2] =
      params->R[2];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.S[2] =
      params->S[2];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.R[3] =
      params->R[3];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace.S[3] =
      params->S[3];
  //  Control change
  //  Initial guess for fmincon:
  //  use the last applied control over the whole prediction horizon
  //  Control limits
  d = u_last_applied[0];
  exitflag = u_last_applied[1];
  for (i = 0; i < 10; i++) {
    colOffsetATw = i << 1;
    U_optimal[colOffsetATw] = d;
    lowerBound[colOffsetATw] = -params->deltaMax;
    U_optimal[colOffsetATw + 1] = exitflag;
    lowerBound[colOffsetATw + 1] = params->aMin;
    upperBound[colOffsetATw] = params->deltaMax;
    upperBound[colOffsetATw + 1] = params->aMax;
  }
  //  @(U_vector) declares U_vector as the input of this function.
  //  fmincon supplies and changes U_vector during optimization.
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .x_current[0] = x_current[0];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .x_current[1] = x_current[1];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .x_current[2] = x_current[2];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .x_current[3] = x_current[3];
  std::copy(&refHorizon[0], &refHorizon[40],
            &FcnEvaluator.next.next.next.next.next.next.next.next.value
                 .workspace.refHorizon[0]);
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .u_last_applied[0] = u_last_applied[0];
  FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
      .u_last_applied[1] = u_last_applied[1];
  //   options = optimoptions( ...
  //      'fmincon', ...
  //     'Algorithm', 'sqp', ...
  //    'Display', 'none');
  //  Find the optimal future control sequence
  exitflag = rtInf;
  i = 0;
  exitg1 = false;
  while ((!exitg1) && (i < 20)) {
    if (lowerBound[i] > upperBound[i]) {
      exitflag = -2.0;
      exitg1 = true;
    } else {
      i++;
    }
  }
  if (!(exitflag == -2.0)) {
    int b_i;
    int mFixed;
    int mLB;
    int mUB;
    TrialState.nVarMax = 21;
    TrialState.mNonlinIneq = 0;
    TrialState.mNonlinEq = 0;
    TrialState.mIneq = 0;
    TrialState.mEq = 0;
    TrialState.iNonIneq0 = 1;
    TrialState.iNonEq0 = 1;
    TrialState.sqpFval_old = 0.0;
    TrialState.sqpIterations = 0;
    TrialState.sqpExitFlag = 0;
    std::memset(&TrialState.lambdasqp[0], 0, 41U * sizeof(double));
    TrialState.steplength = 1.0;
    std::memset(&TrialState.delta_x[0], 0, 21U * sizeof(double));
    TrialState.fstar = 0.0;
    TrialState.firstorderopt = 0.0;
    std::memset(&TrialState.lambda[0], 0, 41U * sizeof(double));
    TrialState.state = 0;
    TrialState.maxConstr = 0.0;
    TrialState.iterations = 0;
    std::copy(&U_optimal[0], &U_optimal[20], &TrialState.xstarsqp[0]);
    WorkingSet.nVar = 20;
    WorkingSet.nVarOrig = 20;
    WorkingSet.nVarMax = 21;
    WorkingSet.ldA = 21;
    std::memset(&WorkingSet.lb[0], 0, 21U * sizeof(double));
    std::memset(&WorkingSet.ub[0], 0, 21U * sizeof(double));
    WorkingSet.mEqRemoved = 0;
    std::memset(&WorkingSet.ATwset[0], 0, 861U * sizeof(double));
    WorkingSet.nActiveConstr = 0;
    std::memset(&WorkingSet.bwset[0], 0, 41U * sizeof(double));
    std::memset(&WorkingSet.maxConstrWorkspace[0], 0, 41U * sizeof(double));
    std::memset(&WorkingSet.Wid[0], 0, 41U * sizeof(int));
    std::memset(&WorkingSet.Wlocalidx[0], 0, 41U * sizeof(int));
    for (i = 0; i < 41; i++) {
      WorkingSet.isActiveConstr[i] = false;
    }
    for (i = 0; i < 5; i++) {
      WorkingSet.nWConstr[i] = 0;
    }
    WorkingSet.probType = 3;
    WorkingSet.SLACK0 = 1.0E-5;
    std::memset(&WorkingSet.indexLB[0], 0, 21U * sizeof(int));
    std::memset(&WorkingSet.indexUB[0], 0, 21U * sizeof(int));
    std::memset(&WorkingSet.indexFixed[0], 0, 21U * sizeof(int));
    mLB = 0;
    mUB = 0;
    mFixed = 0;
    for (colOffsetATw = 0; colOffsetATw < 20; colOffsetATw++) {
      boolean_T guard1;
      d = lowerBound[colOffsetATw];
      guard1 = false;
      if ((!std::isinf(d)) && (!std::isnan(d))) {
        if (std::abs(d - upperBound[colOffsetATw]) < 1.0E-6) {
          mFixed++;
          WorkingSet.indexFixed[mFixed - 1] = colOffsetATw + 1;
        } else {
          mLB++;
          WorkingSet.indexLB[mLB - 1] = colOffsetATw + 1;
          guard1 = true;
        }
      } else {
        guard1 = true;
      }
      if (guard1) {
        d = upperBound[colOffsetATw];
        if ((!std::isinf(d)) && (!std::isnan(d))) {
          mUB++;
          WorkingSet.indexUB[mUB - 1] = colOffsetATw + 1;
        }
      }
    }
    i = (mLB + mUB) + mFixed;
    WorkingSet.mConstr = i;
    WorkingSet.mConstrOrig = i;
    WorkingSet.mConstrMax = 41;
    obj_tmp[0] = static_cast<signed char>(mFixed);
    obj_tmp[1] = 0;
    obj_tmp[2] = 0;
    obj_tmp[3] = static_cast<signed char>(mLB);
    obj_tmp[4] = static_cast<signed char>(mUB);
    b_obj_tmp[0] = static_cast<signed char>(mFixed);
    b_obj_tmp[1] = 0;
    b_obj_tmp[2] = 0;
    b_obj_tmp[3] = static_cast<signed char>(mLB + 1);
    b_obj_tmp[4] = static_cast<signed char>(mUB);
    WorkingSet.isActiveIdx[0] = 1;
    WorkingSet.isActiveIdx[1] = mFixed;
    WorkingSet.isActiveIdx[2] = 0;
    WorkingSet.isActiveIdx[3] = 0;
    WorkingSet.isActiveIdx[4] = mLB;
    WorkingSet.isActiveIdx[5] = mUB;
    for (i = 0; i < 5; i++) {
      signed char i1;
      signed char i2;
      i1 = obj_tmp[i];
      WorkingSet.sizes[i] = i1;
      WorkingSet.sizesNormal[i] = i1;
      i2 = b_obj_tmp[i];
      WorkingSet.sizesPhaseOne[i] = i2;
      WorkingSet.sizesRegularized[i] = i1;
      WorkingSet.sizesRegPhaseOne[i] = i2;
      WorkingSet.isActiveIdx[i + 1] += WorkingSet.isActiveIdx[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxNormal[b_i] = WorkingSet.isActiveIdx[b_i];
    }
    WorkingSet.isActiveIdxPhaseOne[0] = 1;
    WorkingSet.isActiveIdxPhaseOne[1] = mFixed;
    WorkingSet.isActiveIdxPhaseOne[2] = 0;
    WorkingSet.isActiveIdxPhaseOne[3] = 0;
    WorkingSet.isActiveIdxPhaseOne[4] = mLB + 1;
    WorkingSet.isActiveIdxPhaseOne[5] = mUB;
    for (i = 0; i < 5; i++) {
      WorkingSet.isActiveIdxPhaseOne[i + 1] +=
          WorkingSet.isActiveIdxPhaseOne[i];
    }
    for (b_i = 0; b_i < 6; b_i++) {
      WorkingSet.isActiveIdxRegularized[b_i] = WorkingSet.isActiveIdx[b_i];
      WorkingSet.isActiveIdxRegPhaseOne[b_i] =
          WorkingSet.isActiveIdxPhaseOne[b_i];
    }
    for (colOffsetATw = 0; colOffsetATw < mLB; colOffsetATw++) {
      b_i = WorkingSet.indexLB[colOffsetATw];
      TrialState.xstarsqp[b_i - 1] =
          std::fmax(TrialState.xstarsqp[b_i - 1], lowerBound[b_i - 1]);
    }
    for (colOffsetATw = 0; colOffsetATw < mUB; colOffsetATw++) {
      b_i = WorkingSet.indexUB[colOffsetATw];
      TrialState.xstarsqp[b_i - 1] =
          std::fmin(TrialState.xstarsqp[b_i - 1], upperBound[b_i - 1]);
    }
    for (colOffsetATw = 0; colOffsetATw < mFixed; colOffsetATw++) {
      b_i = WorkingSet.indexFixed[colOffsetATw];
      TrialState.xstarsqp[b_i - 1] = upperBound[b_i - 1];
    }
    exitflag = mpcController_anonFcn1(
        x_current, refHorizon, u_last_applied, params->Ts, params->L, params->Q,
        params->R, params->S, TrialState.xstarsqp);
    TrialState.sqpFval = exitflag;
    coder::optim::coder::utils::FiniteDifferences::factoryConstruct(
        FcnEvaluator.next.next.next.next.next.next.next.next.value, lowerBound,
        upperBound, FiniteDifferences);
    coder::optim::coder::utils::FiniteDifferences::internal::
        computeForwardDifferences(FiniteDifferences, exitflag,
                                  TrialState.xstarsqp, TrialState.grad,
                                  lowerBound, upperBound);
    TrialState.FunctionEvaluations = FiniteDifferences.numEvals + 1;
    for (colOffsetATw = 0; colOffsetATw < mLB; colOffsetATw++) {
      WorkingSet.lb[WorkingSet.indexLB[colOffsetATw] - 1] =
          -lowerBound[WorkingSet.indexLB[colOffsetATw] - 1] +
          U_optimal[WorkingSet.indexLB[colOffsetATw] - 1];
    }
    for (colOffsetATw = 0; colOffsetATw < mUB; colOffsetATw++) {
      WorkingSet.ub[WorkingSet.indexUB[colOffsetATw] - 1] =
          upperBound[WorkingSet.indexUB[colOffsetATw] - 1] -
          U_optimal[WorkingSet.indexUB[colOffsetATw] - 1];
    }
    for (colOffsetATw = 0; colOffsetATw < mFixed; colOffsetATw++) {
      d = upperBound[WorkingSet.indexFixed[colOffsetATw] - 1] -
          U_optimal[WorkingSet.indexFixed[colOffsetATw] - 1];
      WorkingSet.ub[WorkingSet.indexFixed[colOffsetATw] - 1] = d;
      WorkingSet.bwset[colOffsetATw] = d;
    }
    coder::optim::coder::qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
    i = WorkingSet.isActiveIdx[2];
    for (colOffsetATw = i; colOffsetATw < 42; colOffsetATw++) {
      WorkingSet.isActiveConstr[colOffsetATw - 1] = false;
    }
    WorkingSet.nWConstr[0] = WorkingSet.sizes[0];
    WorkingSet.nWConstr[1] = 0;
    WorkingSet.nWConstr[2] = 0;
    WorkingSet.nWConstr[3] = 0;
    WorkingSet.nWConstr[4] = 0;
    WorkingSet.nActiveConstr = WorkingSet.nWConstr[0];
    b_i = static_cast<unsigned char>(WorkingSet.sizes[0]);
    for (i = 0; i < b_i; i++) {
      WorkingSet.Wid[i] = 1;
      WorkingSet.Wlocalidx[i] = i + 1;
      WorkingSet.isActiveConstr[i] = true;
      colOffsetATw = 21 * i;
      mLB = WorkingSet.indexFixed[i];
      if (mLB - 2 >= 0) {
        std::memset(&WorkingSet.ATwset[colOffsetATw], 0,
                    static_cast<unsigned int>(
                        ((mLB + colOffsetATw) - colOffsetATw) - 1) *
                        sizeof(double));
      }
      WorkingSet.ATwset[(WorkingSet.indexFixed[i] + colOffsetATw) - 1] = 1.0;
      mLB = WorkingSet.indexFixed[i] + 1;
      mUB = WorkingSet.nVar;
      if (mLB <= mUB) {
        std::memset(&WorkingSet.ATwset[(mLB + colOffsetATw) + -1], 0,
                    static_cast<unsigned int>(
                        (((mUB + colOffsetATw) - mLB) - colOffsetATw) + 1) *
                        sizeof(double));
      }
      WorkingSet.bwset[i] = WorkingSet.ub[WorkingSet.indexFixed[i] - 1];
    }
    double Hessian[400];
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
    coder::optim::coder::fminconsqp::driver(
        lowerBound, upperBound, TrialState, MeritFunction, FcnEvaluator,
        FiniteDifferences, memspace, WorkingSet, Hessian, QRManager,
        CholManager, QPObjective);
    std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[20], &U_optimal[0]);
  }
  //  Receding horizon:
  //  apply only the first optimized control
  u_optimal[0] = U_optimal[0];
  u_optimal[1] = U_optimal[1];
}

double mpcController_anonFcn1(const double x_current[4],
                              const double refHorizon[40],
                              const double u_last_applied[2], double Ts,
                              double L, const double Q[16], const double R[4],
                              const double S[4], const double U_vector[20])
{
  double u_previous_idx_0;
  double u_previous_idx_1;
  double varargout_1;
  double x_predicted_idx_0;
  double x_predicted_idx_1;
  double x_predicted_idx_2;
  double x_predicted_idx_3;
  //  Control sequence currently evaluated by fmincon
  //  Start prediction from current estimated state
  x_predicted_idx_0 = x_current[0];
  x_predicted_idx_1 = x_current[1];
  x_predicted_idx_2 = x_current[2];
  x_predicted_idx_3 = x_current[3];
  //  Previous real control
  u_previous_idx_0 = u_last_applied[0];
  u_previous_idx_1 = u_last_applied[1];
  varargout_1 = 0.0;
  for (int k{0}; k < 10; k++) {
    double b_error[4];
    double c_error;
    double d;
    double x_predicted_idx_3_tmp;
    int error_tmp;
    //  Control at prediction step k
    //  u = [steering angle; acceleration]
    //  Predict next vehicle state
    //  State:   x = [X; Y; psi; v]
    //  Control: u = [delta; a]
    //  Continuous-time kinematic bicycle model
    //  Euler discretization
    x_predicted_idx_0 += Ts * (x_predicted_idx_3 * std::cos(x_predicted_idx_2));
    x_predicted_idx_1 += Ts * (x_predicted_idx_3 * std::sin(x_predicted_idx_2));
    error_tmp = k << 1;
    d = U_vector[error_tmp];
    x_predicted_idx_2 += Ts * (x_predicted_idx_3 / L * std::tan(d));
    x_predicted_idx_3_tmp = U_vector[error_tmp + 1];
    x_predicted_idx_3 += Ts * x_predicted_idx_3_tmp;
    //  Wrap heading to [-pi, pi]
    x_predicted_idx_2 =
        rt_atan2d_snf(std::sin(x_predicted_idx_2), std::cos(x_predicted_idx_2));
    //  Desired state at this future step
    //  Tracking error
    error_tmp = k << 2;
    b_error[0] = x_predicted_idx_0 - refHorizon[error_tmp];
    b_error[1] = x_predicted_idx_1 - refHorizon[error_tmp + 1];
    b_error[2] = x_predicted_idx_2 - refHorizon[error_tmp + 2];
    b_error[3] = x_predicted_idx_3 - refHorizon[error_tmp + 3];
    //  Wrap heading error to [-pi, pi]
    b_error[2] = rt_atan2d_snf(std::sin(b_error[2]), std::cos(b_error[2]));
    //  Change in control
    u_previous_idx_0 = d - u_previous_idx_0;
    u_previous_idx_1 = x_predicted_idx_3_tmp - u_previous_idx_1;
    //  Cost
    c_error = 0.0;
    for (error_tmp = 0; error_tmp < 4; error_tmp++) {
      int i;
      i = error_tmp << 2;
      c_error += (((b_error[0] * Q[i] + b_error[1] * Q[i + 1]) +
                   b_error[2] * Q[i + 2]) +
                  b_error[3] * Q[i + 3]) *
                 b_error[error_tmp];
    }
    varargout_1 =
        ((varargout_1 + c_error) +
         ((d * R[0] + x_predicted_idx_3_tmp * R[1]) * d +
          (d * R[2] + x_predicted_idx_3_tmp * R[3]) * x_predicted_idx_3_tmp)) +
        ((u_previous_idx_0 * S[0] + u_previous_idx_1 * S[1]) *
             u_previous_idx_0 +
         (u_previous_idx_0 * S[2] + u_previous_idx_1 * S[3]) *
             u_previous_idx_1);
    //  For the next predicted step
    u_previous_idx_0 = d;
    u_previous_idx_1 = x_predicted_idx_3_tmp;
  }
  return varargout_1;
}

// End of code generation (mpcController.cpp)
