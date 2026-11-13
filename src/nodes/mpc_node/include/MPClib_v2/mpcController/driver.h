//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// driver.h
//
// Code generation for function 'driver'
//

#ifndef DRIVER_H
#define DRIVER_H

// Include files
#include "rtwtypes.h"
#include <cstddef>
#include <cstdlib>

// Type Declarations
struct g_struct_T;

struct c_struct_T;

struct h_struct_T;

struct d_struct_T;

struct e_struct_T;

struct struct_T;

struct i_struct_T;

// Function Declarations
namespace coder {
namespace optim {
namespace coder {
namespace qpactiveset {
void driver(const double H[400], const double f[21], g_struct_T &solution,
            c_struct_T &memspace, h_struct_T &workingset, d_struct_T &qrmanager,
            e_struct_T &cholmanager, struct_T &objective, i_struct_T &options,
            int runTimeOptions_MaxIterations);

}
} // namespace coder
} // namespace optim
} // namespace coder

#endif
// End of code generation (driver.h)
