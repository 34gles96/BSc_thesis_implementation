//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver1.cpp
//
// Code generation for function 'driver1'
//

// Include files
#include "driver1.h"
#include "BFGSUpdate.h"
#include "computeForwardDifferences.h"
#include "evalObjAndConstr.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "step.h"
#include "stickyStruct.h"
#include "test_exit.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
void driver(const double lb[20], const double ub[20], g_struct_T &TrialState,
            b_struct_T &MeritFunction,
            const ::coder::internal::i_stickyStruct &FcnEvaluator,
            k_struct_T &FiniteDifferences, c_struct_T &memspace,
            h_struct_T &WorkingSet, double Hessian[400],
            d_struct_T &b_QRManager, e_struct_T &b_CholManager,
            struct_T &QPObjective)
{
  static const signed char iv[400]{
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
      1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0,
      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1};
  static const char qpoptions_SolverName[7]{'f', 'm', 'i', 'n', 'c', 'o', 'n'};
  i_struct_T expl_temp;
  j_struct_T Flags;
  int i;
  int ineqStart;
  int ixlast;
  int mConstr;
  int mFixed;
  int mLB;
  int mUB;
  int nVar_tmp_tmp;
  int qpoptions_MaxIterations;
  std::memset(&QPObjective.grad[0], 0, 21U * sizeof(double));
  std::memset(&QPObjective.Hx[0], 0, 20U * sizeof(double));
  QPObjective.hasLinear = true;
  QPObjective.nvar = 20;
  QPObjective.maxVar = 21;
  QPObjective.beta = 0.0;
  QPObjective.rho = 0.0;
  QPObjective.objtype = 3;
  QPObjective.prev_objtype = 3;
  QPObjective.prev_nvar = 0;
  QPObjective.prev_hasLinear = false;
  QPObjective.gammaScalar = 0.0;
  b_CholManager.ldm = 41;
  b_CholManager.ndims = 0;
  b_CholManager.info = 0;
  b_CholManager.scaleFactor = 0.0;
  b_CholManager.ConvexCheck = true;
  b_CholManager.regTol_ = rtInf;
  b_CholManager.workspace_ = rtInf;
  b_CholManager.workspace2_ = rtInf;
  b_QRManager.ldq = 41;
  std::memset(&b_CholManager.FMat[0], 0, 1681U * sizeof(double));
  std::memset(&b_QRManager.QR[0], 0, 1681U * sizeof(double));
  std::memset(&b_QRManager.Q[0], 0, 1681U * sizeof(double));
  b_QRManager.mrows = 0;
  b_QRManager.ncols = 0;
  std::memset(&b_QRManager.jpvt[0], 0, 41U * sizeof(int));
  std::memset(&b_QRManager.tau[0], 0, 41U * sizeof(double));
  b_QRManager.minRowCol = 0;
  b_QRManager.usedPivoting = false;
  for (i = 0; i < 400; i++) {
    Hessian[i] = iv[i];
  }
  nVar_tmp_tmp = WorkingSet.nVar;
  mFixed = WorkingSet.sizes[0];
  mLB = WorkingSet.sizes[3];
  mUB = WorkingSet.sizes[4];
  mConstr = (WorkingSet.sizes[0] + WorkingSet.sizes[3]) + WorkingSet.sizes[4];
  ineqStart = WorkingSet.nVar;
  ixlast =
      (WorkingSet.sizes[3] + WorkingSet.sizes[4]) + (WorkingSet.sizes[0] << 1);
  if (ineqStart >= ixlast) {
    ixlast = ineqStart;
  }
  qpoptions_MaxIterations = 10 * ixlast;
  TrialState.steplength = 1.0;
  test_exit(MeritFunction, WorkingSet, TrialState, lb, ub, Flags.fevalOK,
            Flags.done, Flags.stepAccepted, Flags.failedLineSearch,
            Flags.stepType);
  TrialState.sqpFval_old = TrialState.sqpFval;
  for (int k{0}; k < 20; k++) {
    TrialState.xstarsqp_old[k] = TrialState.xstarsqp[k];
    TrialState.grad_old[k] = TrialState.grad[k];
  }
  if (!Flags.done) {
    TrialState.sqpIterations = 1;
  }
  while (!Flags.done) {
    double phi_alpha;
    while (!(Flags.stepAccepted || Flags.failedLineSearch)) {
      if (Flags.stepType != 3) {
        i = static_cast<unsigned char>(mLB);
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet.lb[WorkingSet.indexLB[ixlast] - 1] =
              -lb[WorkingSet.indexLB[ixlast] - 1] +
              TrialState.xstarsqp[WorkingSet.indexLB[ixlast] - 1];
        }
        i = static_cast<unsigned char>(mUB);
        for (ixlast = 0; ixlast < i; ixlast++) {
          WorkingSet.ub[WorkingSet.indexUB[ixlast] - 1] =
              ub[WorkingSet.indexUB[ixlast] - 1] -
              TrialState.xstarsqp[WorkingSet.indexUB[ixlast] - 1];
        }
        i = static_cast<unsigned char>(mFixed);
        for (ixlast = 0; ixlast < i; ixlast++) {
          phi_alpha = ub[WorkingSet.indexFixed[ixlast] - 1] -
                      TrialState.xstarsqp[WorkingSet.indexFixed[ixlast] - 1];
          WorkingSet.ub[WorkingSet.indexFixed[ixlast] - 1] = phi_alpha;
          WorkingSet.bwset[ixlast] = phi_alpha;
        }
        if (WorkingSet.nActiveConstr > mFixed) {
          ineqStart = mFixed + 1;
          if (ineqStart < 1) {
            ineqStart = 1;
          }
          i = WorkingSet.nActiveConstr;
          for (ixlast = ineqStart; ixlast <= i; ixlast++) {
            switch (WorkingSet.Wid[ixlast - 1]) {
            case 4:
              WorkingSet.bwset[ixlast - 1] =
                  WorkingSet
                      .lb[WorkingSet
                              .indexLB[WorkingSet.Wlocalidx[ixlast - 1] - 1] -
                          1];
              break;
            case 5:
              WorkingSet.bwset[ixlast - 1] =
                  WorkingSet
                      .ub[WorkingSet
                              .indexUB[WorkingSet.Wlocalidx[ixlast - 1] - 1] -
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
      i_struct_T b_expl_temp;
      b_expl_temp = expl_temp;
      Flags.stepAccepted = step(
          Flags.stepType, Hessian, lb, ub, TrialState, MeritFunction, memspace,
          WorkingSet, b_QRManager, b_CholManager, QPObjective, b_expl_temp);
      if (Flags.stepAccepted) {
        i = static_cast<unsigned char>(nVar_tmp_tmp);
        for (ineqStart = 0; ineqStart < i; ineqStart++) {
          TrialState.xstarsqp[ineqStart] += TrialState.delta_x[ineqStart];
        }
        TrialState.sqpFval = utils::ObjNonlinEvaluator::evalObjAndConstr(
            FcnEvaluator, TrialState.xstarsqp, ineqStart);
        Flags.fevalOK = (ineqStart == 1);
        TrialState.FunctionEvaluations++;
        if (Flags.fevalOK) {
          MeritFunction.phiFullStep = TrialState.sqpFval;
        } else {
          MeritFunction.phiFullStep = rtInf;
        }
      }
      if ((Flags.stepType == 1) && Flags.stepAccepted && Flags.fevalOK &&
          (MeritFunction.phi < MeritFunction.phiFullStep) &&
          (TrialState.sqpFval < TrialState.sqpFval_old)) {
        Flags.stepType = 3;
        Flags.stepAccepted = false;
      } else {
        double alpha;
        int exitflagLnSrch;
        boolean_T evalWellDefined;
        boolean_T socTaken;
        if ((Flags.stepType == 3) && Flags.stepAccepted) {
          socTaken = true;
        } else {
          socTaken = false;
        }
        evalWellDefined = Flags.fevalOK;
        i = WorkingSet.nVar;
        alpha = 1.0;
        exitflagLnSrch = 1;
        phi_alpha = MeritFunction.phiFullStep;
        ineqStart = static_cast<unsigned char>(WorkingSet.nVar);
        if (ineqStart - 1 >= 0) {
          std::copy(&TrialState.delta_x[0], &TrialState.delta_x[ineqStart],
                    &TrialState.searchDir[0]);
        }
        int exitg1;
        do {
          exitg1 = 0;
          if (TrialState.FunctionEvaluations < 2000) {
            if (evalWellDefined &&
                (phi_alpha <=
                 MeritFunction.phi +
                     alpha * 0.0001 * MeritFunction.phiPrimePlus)) {
              exitg1 = 1;
            } else {
              boolean_T exitg2;
              boolean_T tooSmallX;
              alpha *= 0.7;
              ineqStart = static_cast<unsigned char>(i);
              for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                TrialState.delta_x[ixlast] = alpha * TrialState.xstar[ixlast];
              }
              if (socTaken) {
                phi_alpha = alpha * alpha;
                if ((i >= 1) && (!(phi_alpha == 0.0))) {
                  ixlast = i - 1;
                  for (int k{0}; k <= ixlast; k++) {
                    TrialState.delta_x[k] +=
                        phi_alpha * TrialState.socDirection[k];
                  }
                }
              }
              tooSmallX = true;
              ixlast = 0;
              exitg2 = false;
              while ((!exitg2) &&
                     (ixlast <= static_cast<unsigned char>(i) - 1)) {
                if (1.0E-6 *
                        std::fmax(1.0, std::abs(TrialState.xstarsqp[ixlast])) <=
                    std::abs(TrialState.delta_x[ixlast])) {
                  tooSmallX = false;
                  exitg2 = true;
                } else {
                  ixlast++;
                }
              }
              if (tooSmallX) {
                exitflagLnSrch = -2;
                exitg1 = 1;
              } else {
                for (ixlast = 0; ixlast < ineqStart; ixlast++) {
                  TrialState.xstarsqp[ixlast] =
                      TrialState.xstarsqp_old[ixlast] +
                      TrialState.delta_x[ixlast];
                }
                TrialState.sqpFval =
                    utils::ObjNonlinEvaluator::evalObjAndConstr(
                        FcnEvaluator, TrialState.xstarsqp, ixlast);
                TrialState.FunctionEvaluations++;
                evalWellDefined = (ixlast == 1);
                if (evalWellDefined) {
                  phi_alpha = TrialState.sqpFval;
                } else {
                  phi_alpha = rtInf;
                }
              }
            }
          } else {
            exitflagLnSrch = 0;
            exitg1 = 1;
          }
        } while (exitg1 == 0);
        Flags.fevalOK = evalWellDefined;
        TrialState.steplength = alpha;
        if (exitflagLnSrch > 0) {
          Flags.stepAccepted = true;
        } else {
          Flags.failedLineSearch = true;
        }
      }
    }
    if (Flags.stepAccepted && (!Flags.failedLineSearch)) {
      i = static_cast<unsigned char>(nVar_tmp_tmp);
      for (ixlast = 0; ixlast < i; ixlast++) {
        TrialState.xstarsqp[ixlast] =
            TrialState.xstarsqp_old[ixlast] + TrialState.delta_x[ixlast];
      }
      i = static_cast<unsigned char>(mConstr);
      for (ixlast = 0; ixlast < i; ixlast++) {
        phi_alpha = TrialState.lambdasqp[ixlast];
        phi_alpha +=
            TrialState.steplength * (TrialState.lambda[ixlast] - phi_alpha);
        TrialState.lambdasqp[ixlast] = phi_alpha;
      }
      TrialState.sqpFval_old = TrialState.sqpFval;
      for (int k{0}; k < 20; k++) {
        TrialState.xstarsqp_old[k] = TrialState.xstarsqp[k];
        TrialState.grad_old[k] = TrialState.grad[k];
      }
      utils::FiniteDifferences::internal::computeForwardDifferences(
          FiniteDifferences, TrialState.sqpFval, TrialState.xstarsqp,
          TrialState.grad, lb, ub);
      TrialState.FunctionEvaluations += FiniteDifferences.numEvals;
    } else {
      TrialState.sqpFval = TrialState.sqpFval_old;
      std::copy(&TrialState.xstarsqp_old[0], &TrialState.xstarsqp_old[20],
                &TrialState.xstarsqp[0]);
    }
    b_test_exit(Flags, memspace, MeritFunction, WorkingSet, TrialState,
                b_QRManager, lb, ub);
    if ((!Flags.done) && Flags.stepAccepted) {
      Flags.stepAccepted = false;
      Flags.stepType = 1;
      Flags.failedLineSearch = false;
      i = static_cast<unsigned char>(nVar_tmp_tmp);
      std::copy(&TrialState.grad[0], &TrialState.grad[i],
                &TrialState.delta_gradLag[0]);
      if (nVar_tmp_tmp >= 1) {
        ixlast = nVar_tmp_tmp - 1;
        for (int k{0}; k <= ixlast; k++) {
          TrialState.delta_gradLag[k] -= TrialState.grad_old[k];
        }
      }
      BFGSUpdate(nVar_tmp_tmp, Hessian, TrialState.delta_x,
                 TrialState.delta_gradLag, memspace.workspace_double);
      TrialState.sqpIterations++;
    }
  }
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (driver1.cpp)
