//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.cpp
//
// Code generation for function 'driver'
//

// Include files
#include "driver.h"
#include "BFGSUpdate.h"
#include "anonymous_function.h"
#include "computeFiniteDifferences.h"
#include "mpcController.h"
#include "mpcController_internal_types.h"
#include "mpcController_internal_types1.h"
#include "rt_nonfinite.h"
#include "step.h"
#include "stickyStruct.h"
#include "test_exit.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(array<double, 2U> &Hessian, const array<double, 1U> &lb,
            const array<double, 1U> &ub, h_struct_T &b_TrialState,
            struct_T &MeritFunction,
            const ::coder::internal::i_stickyStruct &FcnEvaluator,
            k_struct_T &FiniteDifferences, g_struct_T &memspace,
            i_struct_T &WorkingSet, d_struct_T &b_QRManager,
            e_struct_T &b_CholManager, f_struct_T &QPObjective,
            int runTimeOptions_MaxFunctionEvaluations)
{
  static const char qpoptions_SolverName[7]{'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  b_struct_T Flags;
  j_struct_T b_expl_temp;
  j_struct_T expl_temp;
  int ineqStart;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  nVar_tmp_tmp = WorkingSet.nVar - 1;
  mFixed = WorkingSet.sizes[0];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mConstr = (WorkingSet.sizes[0] + WorkingSet.sizes[3]) + WorkingSet.sizes[4];
  nVar = WorkingSet.nVar;
  ineqStart =
      (WorkingSet.sizes[3] + WorkingSet.sizes[4]) + (WorkingSet.sizes[0] << 1);
  if (nVar >= ineqStart) {
    ineqStart = nVar;
  }
  qpoptions_MaxIterations = 10 * ineqStart;
  b_TrialState.steplength = 1.0;
  Flags.gradOK = test_exit(MeritFunction, WorkingSet, b_TrialState, lb, ub,
                           runTimeOptions_MaxFunctionEvaluations, Flags.fevalOK,
                           Flags.done, Flags.stepAccepted,
                           Flags.failedLineSearch, Flags.stepType);
  b_TrialState.sqpFval_old = b_TrialState.sqpFval;
  nVar = b_TrialState.xstarsqp.size(0) - 1;
  for (int k{0}; k <= nVar; k++) {
    b_TrialState.xstarsqp_old[k] = b_TrialState.xstarsqp[k];
    b_TrialState.grad_old[k] = b_TrialState.grad[k];
  }
  if (!Flags.done) {
    b_TrialState.sqpIterations = 1;
  }
  while (!Flags.done) {
    double phi_alpha;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      int i;
      boolean_T hasLB;
      boolean_T hasUB;
      if (Flags.stepType != 3) {
        hasLB = (lb.size(0) != 0);
        hasUB = (ub.size(0) != 0);
        if (hasLB) {
          for (nVar = 0; nVar < mLB; nVar++) {
            WorkingSet.lb[WorkingSet.indexLB[nVar] - 1] =
                -lb[WorkingSet.indexLB[nVar] - 1] +
                b_TrialState.xstarsqp[WorkingSet.indexLB[nVar] - 1];
          }
        }
        if (hasUB) {
          for (nVar = 0; nVar < mUB; nVar++) {
            WorkingSet.ub[WorkingSet.indexUB[nVar] - 1] =
                ub[WorkingSet.indexUB[nVar] - 1] -
                b_TrialState.xstarsqp[WorkingSet.indexUB[nVar] - 1];
          }
        }
        if (hasLB && hasUB) {
          for (nVar = 0; nVar < mFixed; nVar++) {
            phi_alpha = ub[WorkingSet.indexFixed[nVar] - 1] -
                        b_TrialState.xstarsqp[WorkingSet.indexFixed[nVar] - 1];
            WorkingSet.ub[WorkingSet.indexFixed[nVar] - 1] = phi_alpha;
            WorkingSet.bwset[nVar] = phi_alpha;
          }
        }
        if (WorkingSet.nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }
          i = WorkingSet.nActiveConstr;
          for (nVar = ineqStart; nVar <= i; nVar++) {
            switch (WorkingSet.Wid[nVar - 1]) {
            case 4:
              WorkingSet.bwset[nVar - 1] =
                  WorkingSet.lb
                      [WorkingSet.indexLB[WorkingSet.Wlocalidx[nVar - 1] - 1] -
                       1];
              break;
            case 5:
              WorkingSet.bwset[nVar - 1] =
                  WorkingSet.ub
                      [WorkingSet.indexUB[WorkingSet.Wlocalidx[nVar - 1] - 1] -
                       1];
              break;
            default:
              // A check that is always false is detected at compile-time.
              // Eliminating code that follows.
              break;
            }
          }
        }
      }
      expl_temp.ObjectiveLimit = rtMinusInf;
      expl_temp.StepTolerance = 1.0E-6;
      expl_temp.MaxIterations = qpoptions_MaxIterations;
      for (i = 0; i < 7; i++) {
        expl_temp.SolverName[i] = qpoptions_SolverName[i];
      }
      b_expl_temp = expl_temp;
      Flags.stepAccepted =
          b_step(Flags.stepType, Hessian, lb, ub, b_TrialState, MeritFunction,
                 memspace, WorkingSet, b_QRManager, b_CholManager, QPObjective,
                 b_expl_temp);
      if (Flags.stepAccepted) {
        for (nVar = 0; nVar <= nVar_tmp_tmp; nVar++) {
          b_TrialState.xstarsqp[nVar] =
              b_TrialState.xstarsqp[nVar] + b_TrialState.delta_x[nVar];
        }
        phi_alpha = mpcController_anonFcn1(
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .x_current,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .refHorizon,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .u_last_applied,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .Np,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .Ts,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .L,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .Q,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .R,
            FcnEvaluator.next.next.next.next.next.next.next.next.value.workspace
                .S,
            b_TrialState.xstarsqp);
        nVar = 1;
        hasLB = std::isnan(phi_alpha);
        if (std::isinf(phi_alpha) || hasLB) {
          if (hasLB) {
            nVar = -3;
          } else if (phi_alpha < 0.0) {
            nVar = -1;
          } else {
            nVar = -2;
          }
        }
        if (nVar == 1) {
          nVar = 1;
        }
        b_TrialState.sqpFval = phi_alpha;
        Flags.fevalOK = (nVar == 1);
        b_TrialState.FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction.phiFullStep =
              b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
        } else {
          MeritFunction.phiFullStep = rtInf;
        }
      }
      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction.phi < MeritFunction.phiFullStep) &&
          (b_TrialState.sqpFval < b_TrialState.sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        boolean_T socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }
        hasLB = Flags.fevalOK;
        i = WorkingSet.nVar - 1;
        alpha = 1.0;
        ineqStart = 1;
        phi_alpha = MeritFunction.phiFullStep;
        for (int k{0}; k <= i; k++) {
          b_TrialState.searchDir[k] = b_TrialState.delta_x[k];
        }
        int exitg1;
        do {
          exitg1 = 0;
          if (b_TrialState.FunctionEvaluations <
              runTimeOptions_MaxFunctionEvaluations) {
            if (hasLB && (phi_alpha <=
                          MeritFunction.phi +
                              alpha * 0.0001 * MeritFunction.phiPrimePlus)) {
              exitg1 = 1;
            } else {
              boolean_T exitg2;
              alpha *= 0.7;
              for (nVar = 0; nVar <= i; nVar++) {
                b_TrialState.delta_x[nVar] = alpha * b_TrialState.xstar[nVar];
              }
              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i + 1 >= 1) && (!(phi_alpha == 0.0))) {
                  for (int k{0}; k <= i; k++) {
                    b_TrialState.delta_x[k] =
                        b_TrialState.delta_x[k] +
                        phi_alpha * b_TrialState.socDirection[k];
                  }
                }
              }
              hasUB = true;
              nVar = 0;
              exitg2 = false;
              while ((!exitg2) && (nVar <= i)) {
                if (1.0E-6 *
                        std::fmax(1.0, std::abs(b_TrialState.xstarsqp[nVar])) <=
                    std::abs(b_TrialState.delta_x[nVar])) {
                  hasUB = false;
                  exitg2 = true;
                } else {
                  nVar++;
                }
              }
              if (hasUB) {
                ineqStart = -2;
                exitg1 = 1;
              } else {
                for (nVar = 0; nVar <= i; nVar++) {
                  b_TrialState.xstarsqp[nVar] =
                      b_TrialState.xstarsqp_old[nVar] +
                      b_TrialState.delta_x[nVar];
                }
                phi_alpha = mpcController_anonFcn1(
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.x_current,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.refHorizon,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.u_last_applied,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.Np,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.Ts,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.L,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.Q,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.R,
                    FcnEvaluator.next.next.next.next.next.next.next.next.value
                        .workspace.S,
                    b_TrialState.xstarsqp);
                nVar = 1;
                hasLB = std::isnan(phi_alpha);
                if (std::isinf(phi_alpha) || hasLB) {
                  if (hasLB) {
                    nVar = -3;
                  } else if (phi_alpha < 0.0) {
                    nVar = -1;
                  } else {
                    nVar = -2;
                  }
                }
                if (nVar == 1) {
                  nVar = 1;
                }
                b_TrialState.sqpFval = phi_alpha;
                b_TrialState.FunctionEvaluations++;
                hasLB = (nVar == 1);
                if (hasLB) {
                  phi_alpha =
                      b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            ineqStart = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        Flags.fevalOK = hasLB;
        b_TrialState.steplength = alpha;
        if (ineqStart > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }
    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      for (nVar = 0; nVar <= nVar_tmp_tmp; nVar++) {
        b_TrialState.xstarsqp[nVar] =
            b_TrialState.xstarsqp_old[nVar] + b_TrialState.delta_x[nVar];
      }
      for (nVar = 0; nVar < mConstr; nVar++) {
        phi_alpha = b_TrialState.lambdasqp[nVar];
        phi_alpha +=
            b_TrialState.steplength * (b_TrialState.lambda[nVar] - phi_alpha);
        b_TrialState.lambdasqp[nVar] = phi_alpha;
      }
      b_TrialState.sqpFval_old = b_TrialState.sqpFval;
      nVar = b_TrialState.xstarsqp.size(0) - 1;
      for (int k{0}; k <= nVar; k++) {
        b_TrialState.xstarsqp_old[k] = b_TrialState.xstarsqp[k];
        b_TrialState.grad_old[k] = b_TrialState.grad[k];
      }
      Flags.gradOK = utils::FiniteDifferences::computeFiniteDifferences(
          FiniteDifferences, b_TrialState.sqpFval, b_TrialState.xstarsqp,
          b_TrialState.grad, lb, ub);
      b_TrialState.FunctionEvaluations += FiniteDifferences.numEvals;
    } else {
      b_TrialState.sqpFval = b_TrialState.sqpFval_old;
      ineqStart = b_TrialState.xstarsqp.size(0);
      for (int k{0}; k < ineqStart; k++) {
        b_TrialState.xstarsqp[k] = b_TrialState.xstarsqp_old[k];
      }
    }
    b_test_exit(Flags, memspace, MeritFunction, WorkingSet, b_TrialState,
                b_QRManager, lb, ub, runTimeOptions_MaxFunctionEvaluations);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      for (nVar = 0; nVar <= nVar_tmp_tmp; nVar++) {
        b_TrialState.delta_gradLag[nVar] = b_TrialState.grad[nVar];
      }
      if (nVar_tmp_tmp + 1 >= 1) {
        for (int k{0}; k <= nVar_tmp_tmp; k++) {
          b_TrialState.delta_gradLag[k] =
              b_TrialState.delta_gradLag[k] - b_TrialState.grad_old[k];
        }
      }
      BFGSUpdate(nVar_tmp_tmp + 1, Hessian, b_TrialState.delta_x,
                 b_TrialState.delta_gradLag, memspace.workspace_double);
      b_TrialState.sqpIterations++;
    }
  }
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (driver.cpp)
