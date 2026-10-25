//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compressBounds.h
//
// Code generation for function 'compressBounds'
//

#ifndef COMPRESSBOUNDS_H
#define COMPRESSBOUNDS_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
namespace initialize {
int compressBounds(int nVar, array<int, 1U> &indexLB, array<int, 1U> &indexUB,
                   array<int, 1U> &indexFixed, const array<double, 1U> &lb,
                   const array<double, 1U> &ub, int &mUB, int &mFixed);

}
} // namespace qpactiveset
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (compressBounds.h)
