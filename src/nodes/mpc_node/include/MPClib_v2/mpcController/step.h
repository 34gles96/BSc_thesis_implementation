//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// step.h
//
// Code generation for function 'step'
//

#ifndef STEP_H
#define STEP_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct g_struct_T;

struct b_struct_T;

struct c_struct_T;

struct h_struct_T;

struct d_struct_T;

struct e_struct_T;

struct struct_T;

struct i_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
boolean_T step(int &STEP_TYPE, double Hessian[400], const double lb[20],
               const double ub[20], g_struct_T &TrialState,
               b_struct_T &MeritFunction, c_struct_T &memspace,
               h_struct_T &WorkingSet, d_struct_T &b_QRManager,
               e_struct_T &b_CholManager, struct_T &QPObjective,
               i_struct_T &qpoptions);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (step.h)
