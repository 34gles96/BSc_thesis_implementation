//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factor.h
//
// Code generation for function 'factor'
//

#ifndef FACTOR_H
#define FACTOR_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct e_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace DynamicRegCholManager {
void factor(e_struct_T &obj, const array<double, 2U> &A, int ndims, int ldA);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (factor.h)
