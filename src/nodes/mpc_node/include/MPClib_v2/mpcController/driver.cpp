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
#include "PresolveWorkingSet.h"
#include "computeFval.h"
#include "iterate.h"
#include "maxConstraintViolation.h"
#include "mpcController_internal_types.h"
#include "removeConstr.h"
#include "rt_nonfinite.h"
#include "setProblemType.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[400], const double f[21], g_struct_T &solution,
            c_struct_T &memspace, h_struct_T &workingset, d_struct_T &qrmanager,
            e_struct_T &cholmanager, struct_T &objective, i_struct_T &options,
            int runTimeOptions_MaxIterations)
{
  int idxStartIneq;
  int mConstr;
  int nVar_tmp;
  boolean_T guard1;
  solution.iterations = 0;
  nVar_tmp = workingset.nVar;
  guard1 = false;
  if (workingset.probType == 3) {
    mConstr = static_cast<unsigned char>(workingset.sizes[0]);
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      solution.xstar[workingset.indexFixed[idxStartIneq] - 1] =
          workingset.ub[workingset.indexFixed[idxStartIneq] - 1];
    }
    mConstr = static_cast<unsigned char>(workingset.sizes[3]);
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset
              .isActiveConstr[(workingset.isActiveIdx[3] + idxStartIneq) - 1]) {
        solution.xstar[workingset.indexLB[idxStartIneq] - 1] =
            -workingset.lb[workingset.indexLB[idxStartIneq] - 1];
      }
    }
    mConstr = static_cast<unsigned char>(workingset.sizes[4]);
    for (idxStartIneq = 0; idxStartIneq < mConstr; idxStartIneq++) {
      if (workingset
              .isActiveConstr[(workingset.isActiveIdx[4] + idxStartIneq) - 1]) {
        solution.xstar[workingset.indexUB[idxStartIneq] - 1] =
            workingset.ub[workingset.indexUB[idxStartIneq] - 1];
      }
    }
    initialize::PresolveWorkingSet(solution, memspace, workingset, qrmanager);
    if (solution.state >= 0) {
      guard1 = true;
    }
  } else {
    solution.state = 82;
    guard1 = true;
  }
  if (guard1) {
    solution.iterations = 0;
    solution.maxConstr =
        WorkingSet::maxConstraintViolation(workingset, solution.xstar);
    if (solution.maxConstr > 1.0E-6) {
      int PROBTYPE_ORIG;
      int idxEndIneq_tmp_tmp;
      PROBTYPE_ORIG = workingset.probType;
      solution.xstar[20] = solution.maxConstr + 1.0;
      if (workingset.probType == 3) {
        mConstr = 1;
      } else {
        mConstr = 4;
      }
      WorkingSet::setProblemType(workingset, mConstr);
      mConstr = workingset.nWConstr[0] + workingset.nWConstr[1];
      idxStartIneq = mConstr + 1;
      idxEndIneq_tmp_tmp = workingset.nActiveConstr;
      for (int idx_global{idxStartIneq}; idx_global <= idxEndIneq_tmp_tmp;
           idx_global++) {
        workingset.isActiveConstr
            [(workingset.isActiveIdx[workingset.Wid[idx_global - 1] - 1] +
              workingset.Wlocalidx[idx_global - 1]) -
             2] = false;
      }
      workingset.nWConstr[2] = 0;
      workingset.nWConstr[3] = 0;
      workingset.nWConstr[4] = 0;
      workingset.nActiveConstr = mConstr;
      objective.prev_objtype = objective.objtype;
      objective.prev_nvar = objective.nvar;
      objective.prev_hasLinear = objective.hasLinear;
      objective.objtype = 5;
      objective.nvar = 21;
      objective.gammaScalar = 1.0;
      objective.hasLinear = true;
      solution.fstar = Objective::computeFval(
          objective, memspace.workspace_double, H, f, solution.xstar);
      solution.state = 5;
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.SolverName, 1.4901161193847657E-10, 1.0E-6,
              runTimeOptions_MaxIterations);
      if (workingset.isActiveConstr
              [(workingset.isActiveIdx[3] + workingset.sizes[3]) - 2]) {
        boolean_T exitg1;
        idxStartIneq = workingset.sizes[0];
        exitg1 = false;
        while ((!exitg1) && (idxStartIneq + 1 <= workingset.nActiveConstr)) {
          if ((workingset.Wid[idxStartIneq] == 4) &&
              (workingset.Wlocalidx[idxStartIneq] == workingset.sizes[3])) {
            WorkingSet::removeConstr(workingset, idxStartIneq + 1);
            exitg1 = true;
          } else {
            idxStartIneq++;
          }
        }
      }
      mConstr = workingset.nActiveConstr;
      idxStartIneq = workingset.sizes[0];
      while ((mConstr > idxStartIneq) && (mConstr > nVar_tmp)) {
        WorkingSet::removeConstr(workingset, mConstr);
        mConstr--;
      }
      solution.maxConstr = solution.xstar[20];
      WorkingSet::setProblemType(workingset, PROBTYPE_ORIG);
      objective.objtype = objective.prev_objtype;
      objective.nvar = objective.prev_nvar;
      objective.hasLinear = objective.prev_hasLinear;
      options.ObjectiveLimit = rtMinusInf;
      options.StepTolerance = 1.0E-6;
      if (solution.state != 0) {
        solution.maxConstr =
            WorkingSet::maxConstraintViolation(workingset, solution.xstar);
        if (solution.maxConstr > 1.0E-6) {
          std::memset(&solution.lambda[0], 0, 41U * sizeof(double));
          solution.fstar = Objective::computeFval(
              objective, memspace.workspace_double, H, f, solution.xstar);
          solution.state = -2;
        } else {
          if (solution.maxConstr > 0.0) {
            double maxConstr_new;
            mConstr = static_cast<unsigned char>(nVar_tmp);
            if (mConstr - 1 >= 0) {
              std::copy(&solution.xstar[0], &solution.xstar[mConstr],
                        &solution.searchDir[0]);
            }
            initialize::PresolveWorkingSet(solution, memspace, workingset,
                                           qrmanager);
            maxConstr_new =
                WorkingSet::maxConstraintViolation(workingset, solution.xstar);
            if (maxConstr_new >= solution.maxConstr) {
              solution.maxConstr = maxConstr_new;
              if (mConstr - 1 >= 0) {
                std::copy(&solution.searchDir[0], &solution.searchDir[mConstr],
                          &solution.xstar[0]);
              }
            }
          }
          iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
                  objective, options.SolverName, options.StepTolerance,
                  options.ObjectiveLimit, runTimeOptions_MaxIterations);
        }
      }
    } else {
      iterate(H, f, solution, memspace, workingset, qrmanager, cholmanager,
              objective, options.SolverName, options.StepTolerance,
              options.ObjectiveLimit, runTimeOptions_MaxIterations);
    }
  }
}

} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (driver.cpp)
