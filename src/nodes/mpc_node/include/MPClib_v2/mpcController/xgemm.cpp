//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// xgemm.cpp
//
// Code generation for function 'xgemm'
//

// Include files
#include "xgemm.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Definitions
namespace coder {
namespace internal {
namespace blas {
void xgemm(int m, int n, int k, const double A[400], int lda,
           const double B[1681], int ib0, double C[861])
{
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    br = ib0;
    lastColC = 41 * (n - 1);
    for (int cr{0}; cr <= lastColC; cr += 41) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        std::memset(&C[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
      }
    }
    for (int cr{0}; cr <= lastColC; cr += 41) {
      int ar;
      ar = -1;
      i = br + k;
      for (int ib{br}; ib < i; ib++) {
        int i2;
        i1 = cr + 1;
        i2 = cr + m;
        for (int ic{i1}; ic <= i2; ic++) {
          C[ic - 1] += B[ib - 1] * A[(ar + ic) - cr];
        }
        ar += lda;
      }
      br += 41;
    }
  }
}

void xgemm(int m, int n, int k, const double A[1681], int ia0,
           const double B[861], double C[1681])
{
  if ((m != 0) && (n != 0)) {
    int br;
    int i;
    int i1;
    int lastColC;
    lastColC = 41 * (n - 1);
    for (int cr{0}; cr <= lastColC; cr += 41) {
      i = cr + 1;
      i1 = cr + m;
      if (i <= i1) {
        std::memset(&C[i + -1], 0,
                    static_cast<unsigned int>((i1 - i) + 1) * sizeof(double));
      }
    }
    br = -1;
    for (int cr{0}; cr <= lastColC; cr += 41) {
      int ar;
      ar = ia0;
      i = cr + 1;
      i1 = cr + m;
      for (int ic{i}; ic <= i1; ic++) {
        double temp;
        temp = 0.0;
        for (int w{0}; w < k; w++) {
          temp += A[(w + ar) - 1] * B[(w + br) + 1];
        }
        C[ic - 1] += temp;
        ar += 41;
      }
      br += 41;
    }
  }
}

} // namespace blas
} // namespace internal
} // namespace coder

// End of code generation (xgemm.cpp)
