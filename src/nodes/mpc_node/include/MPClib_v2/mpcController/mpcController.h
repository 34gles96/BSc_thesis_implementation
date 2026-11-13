//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController.h
//
// Code generation for function 'mpcController'
//

#ifndef MPCCONTROLLER_H
#define MPCCONTROLLER_H

// Include files
#include "mpcController_types.h"
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
extern void mpcController(const double x_current[4],
                          const double refHorizon[40],
                          const double u_last_applied[2],
                          const struct0_T *params, double u_optimal[2],
                          double U_optimal[20]);

double mpcController_anonFcn1(const double x_current[4],
                              const double refHorizon[40],
                              const double u_last_applied[2], double Ts,
                              double L, const double Q[16], const double R[4],
                              const double S[4], const double U_vector[20]);

#endif
// End of code generation (mpcController.h)
