//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// factoryConstruct.h
//
// Code generation for function 'factoryConstruct'
//

#ifndef FACTORYCONSTRUCT_H
#define FACTORYCONSTRUCT_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct h_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace TrialState {
void factoryConstruct(int nVarMax, int mConstrMax, const array<double, 1U> &x0,
                      h_struct_T &obj);

}
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (factoryConstruct.h)
