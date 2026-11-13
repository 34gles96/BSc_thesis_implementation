//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// computeGradLag.cpp
//
// Code generation for function 'computeGradLag'
//

// Include files
#include "computeGradLag.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cstring>

// Function Definitions
namespace coder {
namespace optim {
namespace coder {
namespace fminconsqp {
namespace stopping {
void b_computeGradLag(double workspace[861], int nVar, const double grad[21],
                      const int finiteFixed[21], int mFixed,
                      const int finiteLB[21], int mLB, const int finiteUB[21],
                      int mUB, const double lambda[41])
{
  int i;
  int i1;
  int iL0;
  i = static_cast<unsigned char>(nVar);
  std::copy(&grad[0], &grad[i], &workspace[0]);
  i = static_cast<unsigned char>(mFixed);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = static_cast<unsigned char>(mLB);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if (static_cast<unsigned char>(mLB) - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + static_cast<unsigned char>(mLB);
  }
  i = static_cast<unsigned char>(mUB);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

void computeGradLag(double workspace[21], int nVar, const double grad[21],
                    const int finiteFixed[21], int mFixed,
                    const int finiteLB[21], int mLB, const int finiteUB[21],
                    int mUB, const double lambda[41])
{
  int i;
  int i1;
  int iL0;
  i = static_cast<unsigned char>(nVar);
  std::copy(&grad[0], &grad[i], &workspace[0]);
  i = static_cast<unsigned char>(mFixed);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteFixed[idx];
    workspace[i1 - 1] += lambda[idx];
  }
  i = static_cast<unsigned char>(mLB);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteLB[idx];
    workspace[i1 - 1] -= lambda[mFixed + idx];
  }
  if (static_cast<unsigned char>(mLB) - 1 < 0) {
    iL0 = mFixed;
  } else {
    iL0 = mFixed + static_cast<unsigned char>(mLB);
  }
  i = static_cast<unsigned char>(mUB);
  for (int idx{0}; idx < i; idx++) {
    i1 = finiteUB[idx];
    workspace[i1 - 1] += lambda[iL0 + idx];
  }
}

} // namespace stopping
} // namespace fminconsqp
} // namespace coder
} // namespace optim
} // namespace coder

// End of code generation (computeGradLag.cpp)
