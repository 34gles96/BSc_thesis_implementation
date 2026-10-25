//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mrdivide_helper.cpp
//
// Code generation for function 'mrdivide_helper'
//

// Include files
#include "mrdivide_helper.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>

// Function Definitions
namespace coder {
namespace internal {
void mrdiv(const double A[12], const double B[9], double Y[12])
{
  double b_A[9];
  double a21;
  double c_Y_tmp;
  double d_Y_tmp;
  double e_Y_tmp;
  double f_Y_tmp;
  double maxval;
  int Y_tmp;
  int b_Y_tmp;
  int r1;
  int r2;
  int r3;
  int rtemp;
  std::copy(&B[0], &B[9], &b_A[0]);
  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = std::abs(B[0]);
  a21 = std::abs(B[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }
  if (std::abs(B[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }
  b_A[r2] = B[r2] / B[r1];
  b_A[r3] /= b_A[r1];
  b_A[r2 + 3] -= b_A[r2] * b_A[r1 + 3];
  b_A[r3 + 3] -= b_A[r3] * b_A[r1 + 3];
  b_A[r2 + 6] -= b_A[r2] * b_A[r1 + 6];
  b_A[r3 + 6] -= b_A[r3] * b_A[r1 + 6];
  if (std::abs(b_A[r3 + 3]) > std::abs(b_A[r2 + 3])) {
    rtemp = r2;
    r2 = r3;
    r3 = rtemp;
  }
  b_A[r3 + 3] /= b_A[r2 + 3];
  b_A[r3 + 6] -= b_A[r3 + 3] * b_A[r2 + 6];
  rtemp = r1 << 2;
  Y[rtemp] = A[0] / b_A[r1];
  Y_tmp = r2 << 2;
  maxval = b_A[r1 + 3];
  Y[Y_tmp] = A[4] - Y[rtemp] * maxval;
  b_Y_tmp = r3 << 2;
  a21 = b_A[r1 + 6];
  Y[b_Y_tmp] = A[8] - Y[rtemp] * a21;
  c_Y_tmp = b_A[r2 + 3];
  Y[Y_tmp] /= c_Y_tmp;
  d_Y_tmp = b_A[r2 + 6];
  Y[b_Y_tmp] -= Y[Y_tmp] * d_Y_tmp;
  e_Y_tmp = b_A[r3 + 6];
  Y[b_Y_tmp] /= e_Y_tmp;
  f_Y_tmp = b_A[r3 + 3];
  Y[Y_tmp] -= Y[b_Y_tmp] * f_Y_tmp;
  Y[rtemp] -= Y[b_Y_tmp] * b_A[r3];
  Y[rtemp] -= Y[Y_tmp] * b_A[r2];
  Y[rtemp + 1] = A[1] / b_A[r1];
  Y[Y_tmp + 1] = A[5] - Y[rtemp + 1] * maxval;
  Y[b_Y_tmp + 1] = A[9] - Y[rtemp + 1] * a21;
  Y[Y_tmp + 1] /= c_Y_tmp;
  Y[b_Y_tmp + 1] -= Y[Y_tmp + 1] * d_Y_tmp;
  Y[b_Y_tmp + 1] /= e_Y_tmp;
  Y[Y_tmp + 1] -= Y[b_Y_tmp + 1] * f_Y_tmp;
  Y[rtemp + 1] -= Y[b_Y_tmp + 1] * b_A[r3];
  Y[rtemp + 1] -= Y[Y_tmp + 1] * b_A[r2];
  Y[rtemp + 2] = A[2] / b_A[r1];
  Y[Y_tmp + 2] = A[6] - Y[rtemp + 2] * maxval;
  Y[b_Y_tmp + 2] = A[10] - Y[rtemp + 2] * a21;
  Y[Y_tmp + 2] /= c_Y_tmp;
  Y[b_Y_tmp + 2] -= Y[Y_tmp + 2] * d_Y_tmp;
  Y[b_Y_tmp + 2] /= e_Y_tmp;
  Y[Y_tmp + 2] -= Y[b_Y_tmp + 2] * f_Y_tmp;
  Y[rtemp + 2] -= Y[b_Y_tmp + 2] * b_A[r3];
  Y[rtemp + 2] -= Y[Y_tmp + 2] * b_A[r2];
  Y[rtemp + 3] = A[3] / b_A[r1];
  Y[Y_tmp + 3] = A[7] - Y[rtemp + 3] * maxval;
  Y[b_Y_tmp + 3] = A[11] - Y[rtemp + 3] * a21;
  Y[Y_tmp + 3] /= c_Y_tmp;
  Y[b_Y_tmp + 3] -= Y[Y_tmp + 3] * d_Y_tmp;
  Y[b_Y_tmp + 3] /= e_Y_tmp;
  Y[Y_tmp + 3] -= Y[b_Y_tmp + 3] * f_Y_tmp;
  Y[rtemp + 3] -= Y[b_Y_tmp + 3] * b_A[r3];
  Y[rtemp + 3] -= Y[Y_tmp + 3] * b_A[r2];
}

void mrdiv(double A[60], const double B[36])
{
  double b_A[36];
  double smax;
  int i;
  int jA;
  int jBcol;
  int jp1j;
  int kBcol;
  signed char ipiv[6];
  std::copy(&B[0], &B[36], &b_A[0]);
  for (i = 0; i < 6; i++) {
    ipiv[i] = static_cast<signed char>(i + 1);
  }
  for (int j{0}; j < 5; j++) {
    int b_tmp;
    int mmj_tmp;
    mmj_tmp = 4 - j;
    b_tmp = j * 7;
    jp1j = b_tmp + 2;
    jA = 6 - j;
    jBcol = 0;
    smax = std::abs(b_A[b_tmp]);
    for (int k{2}; k <= jA; k++) {
      double s;
      s = std::abs(b_A[(b_tmp + k) - 1]);
      if (s > smax) {
        jBcol = k - 1;
        smax = s;
      }
    }
    if (b_A[b_tmp + jBcol] != 0.0) {
      if (jBcol != 0) {
        jA = j + jBcol;
        ipiv[j] = static_cast<signed char>(jA + 1);
        for (int k{0}; k < 6; k++) {
          kBcol = j + k * 6;
          smax = b_A[kBcol];
          jBcol = jA + k * 6;
          b_A[kBcol] = b_A[jBcol];
          b_A[jBcol] = smax;
        }
      }
      i = (b_tmp - j) + 6;
      for (int b_i{jp1j}; b_i <= i; b_i++) {
        b_A[b_i - 1] /= b_A[b_tmp];
      }
    }
    jA = b_tmp;
    for (jBcol = 0; jBcol <= mmj_tmp; jBcol++) {
      smax = b_A[(b_tmp + jBcol * 6) + 6];
      if (smax != 0.0) {
        i = jA + 8;
        jp1j = (jA - j) + 12;
        for (kBcol = i; kBcol <= jp1j; kBcol++) {
          b_A[kBcol - 1] += b_A[((b_tmp + kBcol) - jA) - 7] * -smax;
        }
      }
      jA += 6;
    }
  }
  for (int j{0}; j < 6; j++) {
    jBcol = 10 * j - 1;
    jA = 6 * j;
    for (int k{0}; k < j; k++) {
      kBcol = 10 * k;
      smax = b_A[k + jA];
      if (smax != 0.0) {
        for (int b_i{0}; b_i < 10; b_i++) {
          i = (b_i + jBcol) + 1;
          A[i] -= smax * A[b_i + kBcol];
        }
      }
    }
    smax = 1.0 / b_A[j + jA];
    for (int b_i{0}; b_i < 10; b_i++) {
      i = (b_i + jBcol) + 1;
      A[i] *= smax;
    }
  }
  for (int j{5}; j >= 0; j--) {
    jBcol = 10 * j - 1;
    jA = 6 * j - 1;
    i = j + 2;
    for (int k{i}; k < 7; k++) {
      kBcol = 10 * (k - 1);
      smax = b_A[k + jA];
      if (smax != 0.0) {
        for (int b_i{0}; b_i < 10; b_i++) {
          jp1j = (b_i + jBcol) + 1;
          A[jp1j] -= smax * A[b_i + kBcol];
        }
      }
    }
  }
  for (int j{4}; j >= 0; j--) {
    signed char i1;
    i1 = ipiv[j];
    if (i1 != j + 1) {
      for (int b_i{0}; b_i < 10; b_i++) {
        kBcol = b_i + 10 * j;
        smax = A[kBcol];
        i = b_i + 10 * (i1 - 1);
        A[kBcol] = A[i];
        A[i] = smax;
      }
    }
  }
}

} // namespace internal
} // namespace coder

// End of code generation (mrdivide_helper.cpp)
