//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController_internal_types.h
//
// Code generation for function 'mpcController'
//

#ifndef MPCCONTROLLER_INTERNAL_TYPES_H
#define MPCCONTROLLER_INTERNAL_TYPES_H

// Include files
#include "anonymous_function.h"
#include "mpcController_types.h"
#include "rtwtypes.h"

// Type Definitions
struct struct_T {
  double grad[21];
  double Hx[20];
  boolean_T hasLinear;
  int nvar;
  int maxVar;
  double beta;
  double rho;
  int objtype;
  int prev_objtype;
  int prev_nvar;
  boolean_T prev_hasLinear;
  double gammaScalar;
};

struct b_struct_T {
  double penaltyParam;
  double threshold;
  int nPenaltyDecreases;
  double linearizedConstrViol;
  double initFval;
  double initConstrViolationEq;
  double initConstrViolationIneq;
  double phi;
  double phiPrimePlus;
  double phiFullStep;
  double feasRelativeFactor;
  double nlpPrimalFeasError;
  double nlpDualFeasError;
  double nlpComplError;
  double firstOrderOpt;
  boolean_T hasObjective;
};

struct c_struct_T {
  double workspace_double[861];
  int workspace_int[41];
  int workspace_sort[41];
};

struct d_struct_T {
  int ldq;
  double QR[1681];
  double Q[1681];
  int jpvt[41];
  int mrows;
  int ncols;
  double tau[41];
  int minRowCol;
  boolean_T usedPivoting;
};

struct e_struct_T {
  double FMat[1681];
  int ldm;
  int ndims;
  int info;
  double scaleFactor;
  boolean_T ConvexCheck;
  double regTol_;
  double workspace_;
  double workspace2_;
};

struct g_struct_T {
  int nVarMax;
  int mNonlinIneq;
  int mNonlinEq;
  int mIneq;
  int mEq;
  int iNonIneq0;
  int iNonEq0;
  double sqpFval;
  double sqpFval_old;
  double xstarsqp[20];
  double xstarsqp_old[20];
  double grad[21];
  double grad_old[21];
  int FunctionEvaluations;
  int sqpIterations;
  int sqpExitFlag;
  double lambdasqp[41];
  double lambdaStopTest[41];
  double lambdaStopTestPrev[41];
  double steplength;
  double delta_x[21];
  double socDirection[21];
  int workingset_old[41];
  double gradLag[21];
  double delta_gradLag[21];
  double xstar[21];
  double fstar;
  double firstorderopt;
  double lambda[41];
  int state;
  double maxConstr;
  int iterations;
  double searchDir[21];
};

struct h_struct_T {
  int mConstr;
  int mConstrOrig;
  int mConstrMax;
  int nVar;
  int nVarOrig;
  int nVarMax;
  int ldA;
  double lb[21];
  double ub[21];
  int indexLB[21];
  int indexUB[21];
  int indexFixed[21];
  int mEqRemoved;
  double ATwset[861];
  double bwset[41];
  int nActiveConstr;
  double maxConstrWorkspace[41];
  int sizes[5];
  int sizesNormal[5];
  int sizesPhaseOne[5];
  int sizesRegularized[5];
  int sizesRegPhaseOne[5];
  int isActiveIdx[6];
  int isActiveIdxNormal[6];
  int isActiveIdxPhaseOne[6];
  int isActiveIdxRegularized[6];
  int isActiveIdxRegPhaseOne[6];
  boolean_T isActiveConstr[41];
  int Wid[41];
  int Wlocalidx[41];
  int nWConstr[5];
  int probType;
  double SLACK0;
};

struct i_struct_T {
  char SolverName[7];
  int MaxIterations;
  double StepTolerance;
  double ObjectiveLimit;
};

struct j_struct_T {
  boolean_T fevalOK;
  boolean_T done;
  boolean_T stepAccepted;
  boolean_T failedLineSearch;
  int stepType;
};

struct k_struct_T {
  coder::anonymous_function objfun;
  double f_1;
  int numEvals;
  boolean_T hasLB[20];
  boolean_T hasUB[20];
  boolean_T hasBounds;
};

#endif
// End of code generation (mpcController_internal_types.h)
