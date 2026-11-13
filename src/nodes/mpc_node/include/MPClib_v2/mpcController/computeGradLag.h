//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGradLag.h
//
// Code generation for function 'computeGradLag'
//

#ifndef COMPUTEGRADLAG_H
#define COMPUTEGRADLAG_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void b_computeGradLag(double workspace[861], int nVar, const double grad[21],
                      const int finiteFixed[21], int mFixed,
                      const int finiteLB[21], int mLB, const int finiteUB[21],
                      int mUB, const double lambda[41]);

void computeGradLag(double workspace[21], int nVar, const double grad[21],
                    const int finiteFixed[21], int mFixed,
                    const int finiteLB[21], int mLB, const int finiteUB[21],
                    int mUB, const double lambda[41]);

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (computeGradLag.h)
