//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// step.cpp
//
// Code generation for function 'step'
//

// Include files
#include "step.h"
#include "driver1.h"
#include "mpcController_internal_types.h"
#include "relaxed.h"
#include "rt_nonfinite.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T b_step(int &STEP_TYPE, array<double, 2U> &Hessian,
                 const array<double, 1U> &lb, const array<double, 1U> &ub,
                 h_struct_T &b_TrialState, struct_T &MeritFunction,
                 g_struct_T &memspace, i_struct_T &WorkingSet,
                 d_struct_T &b_QRManager, e_struct_T &b_CholManager,
                 f_struct_T &QPObjective, j_struct_T &qpoptions)
{
  array<double, 1U> r;
  j_struct_T b_qpoptions;
  double oldDirIdx;
  double penaltyParamTrial;
  int iH0;
  int idxEndIneq;
  int mConstrMax;
  int nVar;
  int nVar_tmp_tmp;
  boolean_T checkBoundViolation;
  boolean_T stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet.nVar - 1;
  if (STEP_TYPE != 3) {
    for (int k{0}; k <= nVar_tmp_tmp; k++) {
      b_TrialState.xstar[k] = b_TrialState.xstarsqp[k];
    }
  } else {
    for (int k{0}; k <= nVar_tmp_tmp; k++) {
      b_TrialState.searchDir[k] = b_TrialState.xstar[k];
    }
  }
  int exitg1;
  boolean_T guard1;
  do {
    exitg1 = 0;
    guard1 = false;
    switch (STEP_TYPE) {
    case 1:
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, b_TrialState.grad, b_TrialState, memspace, WorkingSet,
          b_QRManager, b_CholManager, QPObjective, b_qpoptions,
          qpoptions.MaxIterations);
      if (b_TrialState.state > 0) {
        penaltyParamTrial = MeritFunction.penaltyParam;
        oldDirIdx = MeritFunction.linearizedConstrViol;
        MeritFunction.linearizedConstrViol = 0.0;
        if ((oldDirIdx > 2.2204460492503131E-16) &&
            (b_TrialState.fstar > 0.0)) {
          if (b_TrialState.sqpFval == 0.0) {
            penaltyParamTrial = 1.0;
          } else {
            penaltyParamTrial = 1.5;
          }
          penaltyParamTrial =
              penaltyParamTrial * b_TrialState.fstar / oldDirIdx;
        }
        if (penaltyParamTrial < MeritFunction.penaltyParam) {
          MeritFunction.phi = b_TrialState.sqpFval + penaltyParamTrial * 0.0;
          if ((MeritFunction.initFval + penaltyParamTrial * 0.0) -
                  MeritFunction.phi >
              static_cast<double>(MeritFunction.nPenaltyDecreases) *
                  MeritFunction.threshold) {
            MeritFunction.nPenaltyDecreases++;
            if ((MeritFunction.nPenaltyDecreases << 1) >
                b_TrialState.sqpIterations) {
              MeritFunction.threshold *= 10.0;
            }
            MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          } else {
            MeritFunction.phi =
                b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
          }
        } else {
          MeritFunction.penaltyParam = std::fmax(penaltyParamTrial, 1.0E-10);
          MeritFunction.phi =
              b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
        }
        MeritFunction.phiPrimePlus = std::fmin(
            b_TrialState.fstar - MeritFunction.penaltyParam * 0.0, 0.0);
      }
      qpactiveset::parseoutput::sortLambdaQP(
          b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
          memspace.workspace_double);
      if ((b_TrialState.state <= 0) && (b_TrialState.state != -6)) {
        STEP_TYPE = 2;
      } else {
        for (int k{0}; k <= nVar_tmp_tmp; k++) {
          b_TrialState.delta_x[k] = b_TrialState.xstar[k];
        }
        guard1 = true;
      }
      break;
    case 2:
      iH0 = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      mConstrMax = iH0 + 1;
      idxEndIneq = WorkingSet.nActiveConstr;
      for (nVar = mConstrMax; nVar <= idxEndIneq; nVar++) {
        WorkingSet.isActiveConstr
            [(WorkingSet.isActiveIdx[WorkingSet.Wid[nVar - 1] - 1] +
              WorkingSet.Wlocalidx[nVar - 1]) -
             2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = iH0;
      r.set_size(b_TrialState.xstar.size(0));
      iH0 = b_TrialState.xstar.size(0);
      for (nVar = 0; nVar < iH0; nVar++) {
        r[nVar] = b_TrialState.xstar[nVar];
      }
      mConstrMax = WorkingSet.sizes[3] - 1;
      idxEndIneq = WorkingSet.sizes[4] - 1;
      if (lb.size(0) != 0) {
        if (ub.size(0) == 0) {
          for (iH0 = 0; iH0 <= mConstrMax; iH0++) {
            penaltyParamTrial = WorkingSet.lb[WorkingSet.indexLB[iH0] - 1];
            if (-r[WorkingSet.indexLB[iH0] - 1] > penaltyParamTrial) {
              r[WorkingSet.indexLB[iH0] - 1] =
                  -penaltyParamTrial + std::abs(penaltyParamTrial);
            }
          }
        } else {
          for (iH0 = 0; iH0 <= mConstrMax; iH0++) {
            penaltyParamTrial = WorkingSet.lb[WorkingSet.indexLB[iH0] - 1];
            if (-r[WorkingSet.indexLB[iH0] - 1] > penaltyParamTrial) {
              if (std::isinf(ub[WorkingSet.indexLB[iH0] - 1])) {
                r[WorkingSet.indexLB[iH0] - 1] =
                    -penaltyParamTrial + std::abs(penaltyParamTrial);
              } else {
                r[WorkingSet.indexLB[iH0] - 1] =
                    (WorkingSet.ub[WorkingSet.indexLB[iH0] - 1] -
                     penaltyParamTrial) /
                    2.0;
              }
            }
          }
        }
      }
      if (ub.size(0) != 0) {
        if (lb.size(0) == 0) {
          for (iH0 = 0; iH0 <= idxEndIneq; iH0++) {
            penaltyParamTrial = WorkingSet.ub[WorkingSet.indexUB[iH0] - 1];
            if (r[WorkingSet.indexUB[iH0] - 1] > penaltyParamTrial) {
              r[WorkingSet.indexUB[iH0] - 1] =
                  penaltyParamTrial - std::abs(penaltyParamTrial);
            }
          }
        } else {
          for (iH0 = 0; iH0 <= idxEndIneq; iH0++) {
            penaltyParamTrial = WorkingSet.ub[WorkingSet.indexUB[iH0] - 1];
            if (r[WorkingSet.indexUB[iH0] - 1] > penaltyParamTrial) {
              if (std::isinf(lb[WorkingSet.indexUB[iH0] - 1])) {
                r[WorkingSet.indexUB[iH0] - 1] =
                    penaltyParamTrial - std::abs(penaltyParamTrial);
              } else {
                r[WorkingSet.indexUB[iH0] - 1] =
                    (penaltyParamTrial -
                     WorkingSet.lb[WorkingSet.indexUB[iH0] - 1]) /
                    2.0;
              }
            }
          }
        }
      }
      b_TrialState.xstar.set_size(r.size(0));
      iH0 = r.size(0);
      for (nVar = 0; nVar < iH0; nVar++) {
        b_TrialState.xstar[nVar] = r[nVar];
      }
      step::relaxed(Hessian, b_TrialState.grad, b_TrialState, MeritFunction,
                    memspace, WorkingSet, b_QRManager, b_CholManager,
                    QPObjective, qpoptions);
      for (int k{0}; k <= nVar_tmp_tmp; k++) {
        b_TrialState.delta_x[k] = b_TrialState.xstar[k];
      }
      guard1 = true;
      break;
    default:
      nVar = WorkingSet.nVar - 1;
      mConstrMax = WorkingSet.mConstrMax - 1;
      for (int k{0}; k <= nVar; k++) {
        b_TrialState.xstarsqp[k] = b_TrialState.xstarsqp_old[k];
        b_TrialState.socDirection[k] = b_TrialState.xstar[k];
      }
      for (int k{0}; k <= mConstrMax; k++) {
        b_TrialState.lambdaStopTest[k] = b_TrialState.lambda[k];
      }
      for (int k{0}; k <= nVar; k++) {
        b_TrialState.xstar[k] = b_TrialState.xstarsqp[k];
      }
      r.set_size(b_TrialState.grad.size(0));
      iH0 = b_TrialState.grad.size(0);
      for (idxEndIneq = 0; idxEndIneq < iH0; idxEndIneq++) {
        r[idxEndIneq] = b_TrialState.grad[idxEndIneq];
      }
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, r, b_TrialState, memspace, WorkingSet, b_QRManager,
          b_CholManager, QPObjective, b_qpoptions, qpoptions.MaxIterations);
      for (iH0 = 0; iH0 <= nVar; iH0++) {
        penaltyParamTrial = b_TrialState.socDirection[iH0];
        oldDirIdx = penaltyParamTrial;
        penaltyParamTrial = b_TrialState.xstar[iH0] - penaltyParamTrial;
        b_TrialState.socDirection[iH0] = penaltyParamTrial;
        b_TrialState.xstar[iH0] = oldDirIdx;
      }
      stepSuccess =
          (::coder::internal::blas::xnrm2(nVar + 1,
                                          b_TrialState.socDirection) <=
           2.0 * ::coder::internal::blas::xnrm2(nVar + 1, b_TrialState.xstar));
      if (!stepSuccess) {
        for (int k{0}; k <= mConstrMax; k++) {
          b_TrialState.lambda[k] = b_TrialState.lambdaStopTest[k];
        }
      } else {
        qpactiveset::parseoutput::sortLambdaQP(
            b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
            WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
            memspace.workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (b_TrialState.state != -6)) {
        for (iH0 = 0; iH0 <= nVar_tmp_tmp; iH0++) {
          b_TrialState.delta_x[iH0] =
              b_TrialState.xstar[iH0] + b_TrialState.socDirection[iH0];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (b_TrialState.state != -6) {
        exitg1 = 1;
      } else {
        nVar = Hessian.size(0) - 1;
        oldDirIdx = 0.0;
        penaltyParamTrial = 1.0;
        for (iH0 = 0; iH0 <= nVar; iH0++) {
          oldDirIdx = std::fmax(oldDirIdx, std::abs(b_TrialState.grad[iH0]));
          penaltyParamTrial =
              std::fmax(penaltyParamTrial, std::abs(b_TrialState.xstar[iH0]));
        }
        penaltyParamTrial =
            std::fmax(2.2204460492503131E-16, oldDirIdx / penaltyParamTrial);
        for (int idx_col{0}; idx_col <= nVar; idx_col++) {
          iH0 = (nVar + 1) * idx_col;
          for (int k{0}; k < idx_col; k++) {
            Hessian[iH0 + k] = 0.0;
          }
          Hessian[idx_col + Hessian.size(0) * idx_col] = penaltyParamTrial;
          mConstrMax = iH0 + idx_col;
          idxEndIneq = nVar - idx_col;
          for (int k{0}; k < idxEndIneq; k++) {
            Hessian[(mConstrMax + k) + 1] = 0.0;
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    mConstrMax = WorkingSet.sizes[3];
    idxEndIneq = WorkingSet.sizes[4];
    r.set_size(b_TrialState.delta_x.size(0));
    iH0 = b_TrialState.delta_x.size(0);
    for (nVar = 0; nVar < iH0; nVar++) {
      r[nVar] = b_TrialState.delta_x[nVar];
    }
    if (lb.size(0) != 0) {
      for (iH0 = 0; iH0 < mConstrMax; iH0++) {
        oldDirIdx = r[WorkingSet.indexLB[iH0] - 1];
        penaltyParamTrial =
            (b_TrialState.xstarsqp[WorkingSet.indexLB[iH0] - 1] + oldDirIdx) -
            lb[WorkingSet.indexLB[iH0] - 1];
        if (penaltyParamTrial < 0.0) {
          r[WorkingSet.indexLB[iH0] - 1] = oldDirIdx - penaltyParamTrial;
          b_TrialState.xstar[WorkingSet.indexLB[iH0] - 1] =
              b_TrialState.xstar[WorkingSet.indexLB[iH0] - 1] -
              penaltyParamTrial;
        }
      }
    }
    if (ub.size(0) != 0) {
      for (iH0 = 0; iH0 < idxEndIneq; iH0++) {
        oldDirIdx = r[WorkingSet.indexUB[iH0] - 1];
        penaltyParamTrial =
            (ub[WorkingSet.indexUB[iH0] - 1] -
             b_TrialState.xstarsqp[WorkingSet.indexUB[iH0] - 1]) -
            oldDirIdx;
        if (penaltyParamTrial < 0.0) {
          r[WorkingSet.indexUB[iH0] - 1] = oldDirIdx + penaltyParamTrial;
          b_TrialState.xstar[WorkingSet.indexUB[iH0] - 1] =
              b_TrialState.xstar[WorkingSet.indexUB[iH0] - 1] +
              penaltyParamTrial;
        }
      }
    }
    b_TrialState.delta_x.set_size(r.size(0));
    iH0 = r.size(0);
    for (nVar = 0; nVar < iH0; nVar++) {
      b_TrialState.delta_x[nVar] = r[nVar];
    }
  }
  return stepSuccess;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (step.cpp)
