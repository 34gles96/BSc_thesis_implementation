//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// main.cpp
//
// Code generation for function 'main'
//

/*************************************************************************/
/* This automatically generated example C++ main file shows how to call  */
/* entry-point functions that MATLAB Coder generated. You must customize */
/* this file for your application. Do not modify this file directly.     */
/* Instead, make a copy of this file, modify it, and integrate it into   */
/* your development environment.                                         */
/*                                                                       */
/* This file initializes entry-point function arguments to a default     */
/* size and value before calling the entry-point functions. It does      */
/* not store or use any values returned from the entry-point functions.  */
/* If necessary, it does pre-allocate memory for returned values.        */
/* You can use this file as a starting point for a main function that    */
/* you can deploy in your application.                                   */
/*                                                                       */
/* After you copy the file, and before you deploy it, you must make the  */
/* following changes:                                                    */
/* * For variable-size function arguments, change the example sizes to   */
/* the sizes that your application requires.                             */
/* * Change the example values of function arguments to the values that  */
/* your application requires.                                            */
/* * If the entry-point functions return values, store these values or   */
/* otherwise use them as required by your application.                   */
/*                                                                       */
/*************************************************************************/

// Include files
#include "main.h"
#include "mpcController.h"
#include "mpcController_terminate.h"
#include "mpcController_types.h"
#include "rt_nonfinite.h"
#include <cstring>

// Function Declarations
static void argInit_2x1_real_T(double result[2]);

static void argInit_4x10_real_T(double result[40]);

static void argInit_4x1_real_T(double result[4]);

static void argInit_4x4_real_T(double result[16]);

static double argInit_real_T();

static void argInit_struct0_T(struct0_T &result);

// Function Definitions
static void argInit_2x1_real_T(double result[2])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 2; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_4x10_real_T(double result[40])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 40; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

static void argInit_4x1_real_T(double result[4])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 4; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
  }
}

static void argInit_4x4_real_T(double result[16])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 16; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

static double argInit_real_T()
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.Ts = result_tmp;
  result.L = result_tmp;
  argInit_4x1_real_T(result.R);
  result.deltaMax = result_tmp;
  result.aMin = result_tmp;
  result.aMax = result_tmp;
  result.Np = result_tmp;
  argInit_4x4_real_T(result.Q);
  result.S[0] = result.R[0];
  result.S[1] = result.R[1];
  result.S[2] = result.R[2];
  result.S[3] = result.R[3];
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_mpcController();
  // Terminate the application.
  // You do not need to do this more than one time.
  mpcController_terminate();
  return 0;
}

void main_mpcController()
{
  struct0_T r;
  double dv1[40];
  double U_optimal[20];
  double dv[4];
  double dv2[2];
  double u_optimal[2];
  // Initialize function 'mpcController' input arguments.
  // Initialize function input argument 'x_current'.
  // Initialize function input argument 'refHorizon'.
  // Initialize function input argument 'u_last_applied'.
  // Initialize function input argument 'params'.
  // Call the entry-point 'mpcController'.
  argInit_4x1_real_T(dv);
  argInit_4x10_real_T(dv1);
  argInit_2x1_real_T(dv2);
  argInit_struct0_T(r);
  mpcController(dv, dv1, dv2, &r, u_optimal, U_optimal);
}

// End of code generation (main.cpp)
