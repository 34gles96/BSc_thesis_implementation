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
#include "driver.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "xnrm2.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T step(int &STEP_TYPE, double Hessian[400], const double lb[20],
               const double ub[20], g_struct_T &TrialState,
               b_struct_T &MeritFunction, c_struct_T &memspace,
               h_struct_T &WorkingSet, d_struct_T &b_QRManager,
               e_struct_T &b_CholManager, struct_T &QPObjective,
               i_struct_T &qpoptions)
{
  i_struct_T b_qpoptions;
  double dv[21];
  double oldDirIdx;
  double s;
  int idxEndIneq_tmp_tmp;
  int nVar_tmp_tmp;
  boolean_T checkBoundViolation;
  boolean_T stepSuccess;
  stepSuccess = true;
  checkBoundViolation = true;
  nVar_tmp_tmp = WorkingSet.nVar;
  if (STEP_TYPE != 3) {
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.nVar);
    std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[idxEndIneq_tmp_tmp],
              &TrialState.xstar[0]);
  } else {
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.nVar);
    if (idxEndIneq_tmp_tmp - 1 >= 0) {
      std::copy(&TrialState.xstar[0], &TrialState.xstar[idxEndIneq_tmp_tmp],
                &TrialState.searchDir[0]);
    }
  }
  int exitg1;
  boolean_T guard1;
  do {
    int idxStartIneq;
    int temp;
    exitg1 = 0;
    guard1 = false;
    switch (STEP_TYPE) {
    case 1:
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, TrialState.grad, TrialState, memspace, WorkingSet,
          b_QRManager, b_CholManager, QPObjective, b_qpoptions,
          qpoptions.MaxIterations);
      if (TrialState.state > 0) {
        MeritFunction.phi = TrialState.sqpFval;
        MeritFunction.linearizedConstrViol = 0.0;
        MeritFunction.penaltyParam = 1.0;
        MeritFunction.phiPrimePlus = std::fmin(TrialState.fstar, 0.0);
      }
      qpactiveset::parseoutput::sortLambdaQP(
          TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
          memspace.workspace_double);
      if ((TrialState.state <= 0) && (TrialState.state != -6)) {
        STEP_TYPE = 2;
      } else {
        idxEndIneq_tmp_tmp = static_cast<unsigned char>(nVar_tmp_tmp);
        if (idxEndIneq_tmp_tmp - 1 >= 0) {
          std::copy(&TrialState.xstar[0], &TrialState.xstar[idxEndIneq_tmp_tmp],
                    &TrialState.delta_x[0]);
        }
        guard1 = true;
      }
      break;
    case 2: {
      double beta;
      temp = WorkingSet.nWConstr[0] + WorkingSet.nWConstr[1];
      idxStartIneq = temp + 1;
      idxEndIneq_tmp_tmp = WorkingSet.nActiveConstr;
      for (int k{idxStartIneq}; k <= idxEndIneq_tmp_tmp; k++) {
        WorkingSet
            .isActiveConstr[(WorkingSet.isActiveIdx[WorkingSet.Wid[k - 1] - 1] +
                             WorkingSet.Wlocalidx[k - 1]) -
                            2] = false;
      }
      WorkingSet.nWConstr[2] = 0;
      WorkingSet.nWConstr[3] = 0;
      WorkingSet.nWConstr[4] = 0;
      WorkingSet.nActiveConstr = temp;
      std::copy(&TrialState.xstar[0], &TrialState.xstar[21], &dv[0]);
      idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.sizes[3]);
      for (int k{0}; k < idxEndIneq_tmp_tmp; k++) {
        s = WorkingSet.lb[WorkingSet.indexLB[k] - 1];
        if (-dv[WorkingSet.indexLB[k] - 1] > s) {
          if (std::isinf(ub[WorkingSet.indexLB[k] - 1])) {
            dv[WorkingSet.indexLB[k] - 1] = -s + std::abs(s);
          } else {
            dv[WorkingSet.indexLB[k] - 1] =
                (WorkingSet.ub[WorkingSet.indexLB[k] - 1] - s) / 2.0;
          }
        }
      }
      idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.sizes[4]);
      for (int k{0}; k < idxEndIneq_tmp_tmp; k++) {
        s = WorkingSet.ub[WorkingSet.indexUB[k] - 1];
        if (dv[WorkingSet.indexUB[k] - 1] > s) {
          if (std::isinf(lb[WorkingSet.indexUB[k] - 1])) {
            dv[WorkingSet.indexUB[k] - 1] = s - std::abs(s);
          } else {
            dv[WorkingSet.indexUB[k] - 1] =
                (s - WorkingSet.lb[WorkingSet.indexUB[k] - 1]) / 2.0;
          }
        }
      }
      std::copy(&dv[0], &dv[21], &TrialState.xstar[0]);
      idxEndIneq_tmp_tmp = WorkingSet.nVar;
      beta = 0.0;
      idxStartIneq = static_cast<unsigned char>(WorkingSet.nVar);
      for (int k{0}; k < idxStartIneq; k++) {
        beta += Hessian[k + 20 * k];
      }
      beta /= static_cast<double>(WorkingSet.nVar);
      if (TrialState.sqpIterations <= 1) {
        temp = QPObjective.nvar;
        if (QPObjective.nvar < 1) {
          idxStartIneq = 0;
        } else {
          idxStartIneq = 1;
          if (QPObjective.nvar > 1) {
            oldDirIdx = std::abs(TrialState.grad[0]);
            for (int k{2}; k <= temp; k++) {
              s = std::abs(TrialState.grad[k - 1]);
              if (s > oldDirIdx) {
                idxStartIneq = k;
                oldDirIdx = s;
              }
            }
          }
        }
        s = 100.0 * std::fmax(1.0, std::abs(TrialState.grad[idxStartIneq - 1]));
      } else {
        temp = WorkingSet.mConstr;
        if (WorkingSet.mConstr < 1) {
          idxStartIneq = 0;
        } else {
          idxStartIneq = 1;
          if (WorkingSet.mConstr > 1) {
            oldDirIdx = std::abs(TrialState.lambdasqp[0]);
            for (int k{2}; k <= temp; k++) {
              s = std::abs(TrialState.lambdasqp[k - 1]);
              if (s > oldDirIdx) {
                idxStartIneq = k;
                oldDirIdx = s;
              }
            }
          }
        }
        s = std::abs(TrialState.lambdasqp[idxStartIneq - 1]);
      }
      QPObjective.nvar = WorkingSet.nVar;
      QPObjective.beta = beta;
      QPObjective.rho = s;
      QPObjective.hasLinear = true;
      QPObjective.objtype = 4;
      qpactiveset::WorkingSet::setProblemType(WorkingSet, 2);
      temp = qpoptions.MaxIterations;
      qpoptions.MaxIterations =
          (qpoptions.MaxIterations + WorkingSet.nVar) - idxEndIneq_tmp_tmp;
      std::copy(&TrialState.grad[0], &TrialState.grad[21], &dv[0]);
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, dv, TrialState, memspace, WorkingSet, b_QRManager,
          b_CholManager, QPObjective, b_qpoptions, qpoptions.MaxIterations);
      qpoptions.MaxIterations = temp;
      if (TrialState.state != -6) {
        MeritFunction.phi = TrialState.sqpFval;
        MeritFunction.linearizedConstrViol = 0.0;
        MeritFunction.penaltyParam = 1.0;
        MeritFunction.phiPrimePlus =
            std::fmin((TrialState.fstar - s * 0.0) - beta / 2.0 * 0.0, 0.0);
        temp = WorkingSet.isActiveIdx[2];
        idxStartIneq = WorkingSet.nActiveConstr;
        for (int k{temp}; k <= idxStartIneq; k++) {
          if (WorkingSet.Wid[k - 1] == 3) {
            TrialState.lambda[k - 1] *= static_cast<double>(
                memspace.workspace_int[WorkingSet.Wlocalidx[k - 1] - 1]);
          }
        }
      }
      QPObjective.nvar = idxEndIneq_tmp_tmp;
      QPObjective.hasLinear = true;
      QPObjective.objtype = 3;
      qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
      qpactiveset::parseoutput::sortLambdaQP(
          TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
          WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
          memspace.workspace_double);
      idxEndIneq_tmp_tmp = static_cast<unsigned char>(nVar_tmp_tmp);
      if (idxEndIneq_tmp_tmp - 1 >= 0) {
        std::copy(&TrialState.xstar[0], &TrialState.xstar[idxEndIneq_tmp_tmp],
                  &TrialState.delta_x[0]);
      }
      guard1 = true;
    } break;
    default:
      idxEndIneq_tmp_tmp = WorkingSet.nVar;
      idxStartIneq = static_cast<unsigned char>(WorkingSet.nVar);
      for (int k{0}; k < idxStartIneq; k++) {
        TrialState.xstarsqp[k] = TrialState.xstarsqp_old[k];
        TrialState.socDirection[k] = TrialState.xstar[k];
      }
      std::copy(&TrialState.lambda[0], &TrialState.lambda[41],
                &TrialState.lambdaStopTest[0]);
      std::copy(&TrialState.xstarsqp[0], &TrialState.xstarsqp[idxStartIneq],
                &TrialState.xstar[0]);
      std::copy(&TrialState.grad[0], &TrialState.grad[21], &dv[0]);
      b_qpoptions = qpoptions;
      ::coder::optim::coder::qpactiveset::driver(
          Hessian, dv, TrialState, memspace, WorkingSet, b_QRManager,
          b_CholManager, QPObjective, b_qpoptions, qpoptions.MaxIterations);
      idxStartIneq = static_cast<unsigned char>(idxEndIneq_tmp_tmp);
      for (int k{0}; k < idxStartIneq; k++) {
        s = TrialState.socDirection[k];
        oldDirIdx = s;
        s = TrialState.xstar[k] - s;
        TrialState.socDirection[k] = s;
        TrialState.xstar[k] = oldDirIdx;
      }
      stepSuccess = (::coder::internal::blas::xnrm2(idxEndIneq_tmp_tmp,
                                                    TrialState.socDirection) <=
                     2.0 * ::coder::internal::blas::xnrm2(idxEndIneq_tmp_tmp,
                                                          TrialState.xstar));
      if (!stepSuccess) {
        std::copy(&TrialState.lambdaStopTest[0], &TrialState.lambdaStopTest[41],
                  &TrialState.lambda[0]);
      } else {
        qpactiveset::parseoutput::sortLambdaQP(
            TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
            WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
            memspace.workspace_double);
      }
      checkBoundViolation = stepSuccess;
      if (stepSuccess && (TrialState.state != -6)) {
        idxEndIneq_tmp_tmp = static_cast<unsigned char>(nVar_tmp_tmp);
        for (int k{0}; k < idxEndIneq_tmp_tmp; k++) {
          TrialState.delta_x[k] =
              TrialState.xstar[k] + TrialState.socDirection[k];
        }
      }
      guard1 = true;
      break;
    }
    if (guard1) {
      if (TrialState.state != -6) {
        exitg1 = 1;
      } else {
        oldDirIdx = 0.0;
        s = 1.0;
        for (int k{0}; k < 20; k++) {
          oldDirIdx = std::fmax(oldDirIdx, std::abs(TrialState.grad[k]));
          s = std::fmax(s, std::abs(TrialState.xstar[k]));
        }
        s = std::fmax(2.2204460492503131E-16, oldDirIdx / s);
        for (idxStartIneq = 0; idxStartIneq < 20; idxStartIneq++) {
          temp = 20 * idxStartIneq;
          for (int k{0}; k < idxStartIneq; k++) {
            Hessian[temp + k] = 0.0;
          }
          temp = idxStartIneq + 20 * idxStartIneq;
          Hessian[temp] = s;
          idxEndIneq_tmp_tmp = 18 - idxStartIneq;
          if (idxEndIneq_tmp_tmp >= 0) {
            std::memset(&Hessian[temp + 1], 0,
                        static_cast<unsigned int>(
                            ((idxEndIneq_tmp_tmp + temp) - temp) + 1) *
                            sizeof(double));
          }
        }
      }
    }
  } while (exitg1 == 0);
  if (checkBoundViolation) {
    std::copy(&TrialState.delta_x[0], &TrialState.delta_x[21], &dv[0]);
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.sizes[3]);
    for (int k{0}; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet.indexLB[k] - 1];
      s = (TrialState.xstarsqp[WorkingSet.indexLB[k] - 1] + oldDirIdx) -
          lb[WorkingSet.indexLB[k] - 1];
      if (s < 0.0) {
        dv[WorkingSet.indexLB[k] - 1] = oldDirIdx - s;
        TrialState.xstar[WorkingSet.indexLB[k] - 1] -= s;
      }
    }
    idxEndIneq_tmp_tmp = static_cast<unsigned char>(WorkingSet.sizes[4]);
    for (int k{0}; k < idxEndIneq_tmp_tmp; k++) {
      oldDirIdx = dv[WorkingSet.indexUB[k] - 1];
      s = (ub[WorkingSet.indexUB[k] - 1] -
           TrialState.xstarsqp[WorkingSet.indexUB[k] - 1]) -
          oldDirIdx;
      if (s < 0.0) {
        dv[WorkingSet.indexUB[k] - 1] = oldDirIdx + s;
        TrialState.xstar[WorkingSet.indexUB[k] - 1] += s;
      }
    }
    std::copy(&dv[0], &dv[21], &TrialState.delta_x[0]);
  }
  return stepSuccess;
}

} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (step.cpp)
