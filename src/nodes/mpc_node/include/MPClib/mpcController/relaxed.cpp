//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// relaxed.cpp
//
// Code generation for function 'relaxed'
//

// Include files
#include "relaxed.h"
#include "driver1.h"
#include "mpcController_internal_types.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include "sortLambdaQP.h"
#include "coder_array.h"
#include <cmath>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace step {
void relaxed(const array<double, 2U> &Hessian, const array<double, 1U> &grad,
             h_struct_T &b_TrialState, struct_T &MeritFunction,
             g_struct_T &memspace, i_struct_T &WorkingSet,
             d_struct_T &b_QRManager, e_struct_T &b_CholManager,
             f_struct_T &QPObjective, j_struct_T &qpoptions)
{
  j_struct_T b_qpoptions;
  double beta;
  double s;
  double smax;
  int idx;
  int idx_max;
  int nVarOrig;
  int temp;
  nVarOrig = WorkingSet.nVar;
  beta = 0.0;
  for (idx = 0; idx < nVarOrig; idx++) {
    beta += Hessian[idx + Hessian.size(0) * idx];
  }
  beta /= static_cast<double>(WorkingSet.nVar);
  if (b_TrialState.sqpIterations <= 1) {
    temp = QPObjective.nvar;
    if (QPObjective.nvar < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (QPObjective.nvar > 1) {
        smax = std::abs(grad[0]);
        for (int k{2}; k <= temp; k++) {
          s = std::abs(grad[k - 1]);
          if (s > smax) {
            idx_max = k;
            smax = s;
          }
        }
      }
    }
    smax = 100.0 * std::fmax(1.0, std::abs(grad[idx_max - 1]));
  } else {
    temp = WorkingSet.mConstr;
    if (WorkingSet.mConstr < 1) {
      idx_max = 0;
    } else {
      idx_max = 1;
      if (WorkingSet.mConstr > 1) {
        smax = std::abs(b_TrialState.lambdasqp[0]);
        for (int k{2}; k <= temp; k++) {
          s = std::abs(b_TrialState.lambdasqp[k - 1]);
          if (s > smax) {
            idx_max = k;
            smax = s;
          }
        }
      }
    }
    smax = std::abs(b_TrialState.lambdasqp[idx_max - 1]);
  }
  QPObjective.nvar = WorkingSet.nVar;
  QPObjective.beta = beta;
  QPObjective.rho = smax;
  QPObjective.hasLinear = true;
  QPObjective.objtype = 4;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 2);
  temp = qpoptions.MaxIterations;
  qpoptions.MaxIterations =
      (qpoptions.MaxIterations + WorkingSet.nVar) - nVarOrig;
  b_qpoptions = qpoptions;
  ::coder::optim::coder::qpactiveset::driver(
      Hessian, grad, b_TrialState, memspace, WorkingSet, b_QRManager,
      b_CholManager, QPObjective, b_qpoptions, qpoptions.MaxIterations);
  qpoptions.MaxIterations = temp;
  if (b_TrialState.state != -6) {
    double qpfvalQuadExcess;
    idx_max = (WorkingSet.nVarMax - nVarOrig) - 1;
    idx = nVarOrig + 1;
    s = 0.0;
    qpfvalQuadExcess = 0.0;
    if (idx_max >= 1) {
      temp = nVarOrig + idx_max;
      for (int k{idx}; k <= temp; k++) {
        s += std::abs(b_TrialState.xstar[k - 1]);
      }
    }
    if (idx_max >= 1) {
      for (int k{0}; k < idx_max; k++) {
        temp = nVarOrig + k;
        qpfvalQuadExcess += b_TrialState.xstar[temp] * b_TrialState.xstar[temp];
      }
    }
    qpfvalQuadExcess =
        (b_TrialState.fstar - smax * s) - beta / 2.0 * qpfvalQuadExcess;
    beta = MeritFunction.penaltyParam;
    smax = MeritFunction.linearizedConstrViol;
    s = 0.0;
    if (idx_max >= 1) {
      temp = nVarOrig + idx_max;
      for (int k{idx}; k <= temp; k++) {
        s += std::abs(b_TrialState.xstar[k - 1]);
      }
    }
    MeritFunction.linearizedConstrViol = s;
    smax -= s;
    if ((smax > 2.2204460492503131E-16) && (qpfvalQuadExcess > 0.0)) {
      if (b_TrialState.sqpFval == 0.0) {
        beta = 1.0;
      } else {
        beta = 1.5;
      }
      beta = beta * qpfvalQuadExcess / smax;
    }
    if (beta < MeritFunction.penaltyParam) {
      MeritFunction.phi = b_TrialState.sqpFval + beta * 0.0;
      if ((MeritFunction.initFval + beta * 0.0) - MeritFunction.phi >
          static_cast<double>(MeritFunction.nPenaltyDecreases) *
              MeritFunction.threshold) {
        MeritFunction.nPenaltyDecreases++;
        if ((MeritFunction.nPenaltyDecreases << 1) >
            b_TrialState.sqpIterations) {
          MeritFunction.threshold *= 10.0;
        }
        MeritFunction.penaltyParam = std::fmax(beta, 1.0E-10);
      } else {
        MeritFunction.phi =
            b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
      }
    } else {
      MeritFunction.penaltyParam = std::fmax(beta, 1.0E-10);
      MeritFunction.phi =
          b_TrialState.sqpFval + MeritFunction.penaltyParam * 0.0;
    }
    MeritFunction.phiPrimePlus =
        std::fmin(qpfvalQuadExcess - MeritFunction.penaltyParam * 0.0, 0.0);
    temp = WorkingSet.isActiveIdx[2];
    idx_max = WorkingSet.nActiveConstr;
    for (idx = temp; idx <= idx_max; idx++) {
      if (WorkingSet.Wid[idx - 1] == 3) {
        b_TrialState.lambda[idx - 1] =
            static_cast<double>(
                memspace.workspace_int[WorkingSet.Wlocalidx[idx - 1] - 1]) *
            b_TrialState.lambda[idx - 1];
      }
    }
  }
  QPObjective.nvar = nVarOrig;
  QPObjective.hasLinear = true;
  QPObjective.objtype = 3;
  qpactiveset::WorkingSet::setProblemType(WorkingSet, 3);
  qpactiveset::parseoutput::sortLambdaQP(
      b_TrialState.lambda, WorkingSet.nActiveConstr, WorkingSet.sizes,
      WorkingSet.isActiveIdx, WorkingSet.Wid, WorkingSet.Wlocalidx,
      memspace.workspace_double);
}

} // namespace step
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (relaxed.cpp)
