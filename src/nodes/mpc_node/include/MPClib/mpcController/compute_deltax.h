//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// compute_deltax.h
//
// Code generation for function 'compute_deltax'
//

#ifndef COMPUTE_DELTAX_H
#define COMPUTE_DELTAX_H

// Include files
#include "rtwtypes.h"
#include "coder_array.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct h_struct_T;

struct g_struct_T;

struct d_struct_T;

struct e_struct_T;

struct f_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void compute_deltax(const array<double, 2U> &H, h_struct_T &solution,
                    g_struct_T &memspace, const d_struct_T &qrmanager,
                    e_struct_T &cholmanager, const f_struct_T &objective,
                    boolean_T alwaysPositiveDef);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (compute_deltax.h)
