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
#include "EKF_func.h"
#include "EKF_func_terminate.h"
#include "EKF_func_types.h"
#include "rt_nonfinite.h"
#include <algorithm>

// Function Declarations
static void argInit_10x10_real_T(double result[100]);

static void argInit_3x1_real_T(double result[3]);

static void argInit_4x1_real_T(double result[4]);

static boolean_T argInit_boolean_T();

static double argInit_real_T();

static void argInit_struct0_T(struct0_T &result);

static void argInit_struct1_T(struct1_T &result);

static void argInit_struct2_T(struct2_T &result);

static struct3_T argInit_struct3_T();

static void argInit_struct4_T(struct4_T &result);

// Function Definitions
static void argInit_10x10_real_T(double result[100])
{
  // Loop over the array to initialize each element.
  for (int i{0}; i < 100; i++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[i] = argInit_real_T();
  }
}

static void argInit_3x1_real_T(double result[3])
{
  // Loop over the array to initialize each element.
  for (int idx0{0}; idx0 < 3; idx0++) {
    // Set the value of the array element.
    // Change this value to the value that the application requires.
    result[idx0] = argInit_real_T();
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

static boolean_T argInit_boolean_T()
{
  return false;
}

static double argInit_real_T()
{
  return 0.0;
}

static void argInit_struct0_T(struct0_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result.valid = argInit_boolean_T();
  result.timestamp = argInit_real_T();
  argInit_3x1_real_T(result.gyro);
  argInit_struct1_T(result.mounting);
}

static void argInit_struct1_T(struct1_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result.x_sensor_body);
  result.y_sensor_body[0] = result.x_sensor_body[0];
  result.z_sensor_body[0] = result.x_sensor_body[0];
  result.y_sensor_body[1] = result.x_sensor_body[1];
  result.z_sensor_body[1] = result.x_sensor_body[1];
  result.y_sensor_body[2] = result.x_sensor_body[2];
  result.z_sensor_body[2] = result.x_sensor_body[2];
}

static void argInit_struct2_T(struct2_T &result)
{
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result.lla1);
  result_tmp = argInit_real_T();
  result.hAcc1 = result_tmp;
  result.vAcc1 = result_tmp;
  result.hAcc2 = result_tmp;
  result.vAcc2 = result_tmp;
  result.valid = argInit_boolean_T();
  result.timestamp = result_tmp;
  result.lla2[0] = result.lla1[0];
  result.position1_enu[0] = result.lla1[0];
  result.position2_enu[0] = result.lla1[0];
  result.position_enu[0] = result.lla1[0];
  result.baseline_enu[0] = result.lla1[0];
  result.baseline_body[0] = result.lla1[0];
  result.lla2[1] = result.lla1[1];
  result.position1_enu[1] = result.lla1[1];
  result.position2_enu[1] = result.lla1[1];
  result.position_enu[1] = result.lla1[1];
  result.baseline_enu[1] = result.lla1[1];
  result.baseline_body[1] = result.lla1[1];
  result.lla2[2] = result.lla1[2];
  result.position1_enu[2] = result.lla1[2];
  result.position2_enu[2] = result.lla1[2];
  result.position_enu[2] = result.lla1[2];
  result.baseline_enu[2] = result.lla1[2];
  result.baseline_body[2] = result.lla1[2];
}

static struct3_T argInit_struct3_T()
{
  struct3_T result;
  double result_tmp;
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  result_tmp = argInit_real_T();
  result.steering_angle = result_tmp;
  result.wheelbase = result_tmp;
  result.velocity = result_tmp;
  return result;
}

static void argInit_struct4_T(struct4_T &result)
{
  // Set the value of each structure field.
  // Change this value to the value that the application requires.
  argInit_3x1_real_T(result.p);
  argInit_10x10_real_T(result.P);
  argInit_4x1_real_T(result.q);
  result.bg[0] = result.p[0];
  result.bg[1] = result.p[1];
  result.bg[2] = result.p[2];
  std::copy(&result.P[0], &result.P[100], &result.Q[0]);
  result.p_ecef0[0] = result.p[0];
  result.p_ecef0[1] = result.p[1];
  result.p_ecef0[2] = result.p[2];
}

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_EKF_func();
  // Terminate the application.
  // You do not need to do this more than one time.
  EKF_func_terminate();
  return 0;
}

void main_EKF_func()
{
  struct0_T r;
  struct2_T r1;
  struct3_T r2;
  struct4_T state;
  // Initialize function 'EKF_func' input arguments.
  // Initialize function input argument 'IMU'.
  // Initialize function input argument 'GNSS'.
  // Initialize function input argument 'Vehicle'.
  // Initialize function input argument 'state'.
  // Call the entry-point 'EKF_func'.
  argInit_struct4_T(state);
  argInit_struct0_T(r);
  argInit_struct2_T(r1);
  r2 = argInit_struct3_T();
  EKF_func(&r, &r1, &r2, &state);
}

// End of code generation (main.cpp)
