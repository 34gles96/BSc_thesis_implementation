//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemv.cpp
//
// Code generation for function 'xgemv'
//

// Include files
#include "xgemv.h"
#include "mpcController_rtwutil.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemv(int m, int n, const double A[400], int lda, const double x[21],
           double y[20])
{
  if ((m != 0) && (n != 0)) {
    int i;
    int ix;
    i = static_cast<unsigned char>(m);
    std::memset(&y[0], 0, static_cast<unsigned int>(i) * sizeof(double));
    ix = 0;
    i = lda * (n - 1) + 1;
    for (int iac{1}; lda < 0 ? iac >= i : iac <= i; iac += lda) {
      int i1;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        int i2;
        i2 = ia - iac;
        y[i2] += A[ia - 1] * x[ix];
      }
      ix++;
    }
  }
}

void xgemv(int m, int n, const double A[1681], const double x[21],
           double y[861])
{
  if (m != 0) {
    int i;
    std::memset(&y[0], 0, static_cast<unsigned int>(n) * sizeof(double));
    i = 41 * (n - 1) + 1;
    for (int iac{1}; iac <= i; iac += 41) {
      double c;
      int i1;
      c = 0.0;
      i1 = (iac + m) - 1;
      for (int ia{iac}; ia <= i1; ia++) {
        c += A[ia - 1] * x[ia - iac];
      }
      i1 = div_nde_s32_floor(iac - 1, 41);
      y[i1] += c;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemv.cpp)
