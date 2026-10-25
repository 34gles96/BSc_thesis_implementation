//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// fmincon.h
//
// Code generation for function 'fmincon'
//

#ifndef FMINCON_H
#define FMINCON_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
namespace coder {
class anonymous_function;

}

// Function Declarations
namespace coder {
void fmincon(const anonymous_function &fun, array<double, 1U> &x0,
             const array<double, 1U> &lb, const array<double, 1U> &ub);

}

#endif
// End of code generation (fmincon.h)
