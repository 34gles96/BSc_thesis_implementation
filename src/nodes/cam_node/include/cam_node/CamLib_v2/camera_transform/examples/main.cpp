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
#include "camera_transform_terminate.h"
#include "initializeCameraToCoGTransform.h"
#include "transformBodyToENU.h"
#include "transformCameraToCoG.h"

// Function Declarations
static void argInit_3x1_real_T(double result[3]);

static void argInit_4x1_real_T(double result[4]);

static void argInit_4x4_real_T(double result[16]);

static double argInit_real_T();

// Function Definitions
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

int main(int, char **)
{
  // The initialize function is being called automatically from your entry-point
  // function. So, a call to initialize is not included here. Invoke the
  // entry-point functions.
  // You can call entry-point functions multiple times.
  main_initializeCameraToCoGTransform();
  main_transformCameraToCoG();
  main_transformBodyToENU();
  // Terminate the application.
  // You do not need to do this more than one time.
  camera_transform_terminate();
  return 0;
}

void main_initializeCameraToCoGTransform()
{
  double T_body_camera[16];
  double camera_pos_V_tmp[3];
  // Initialize function 'initializeCameraToCoGTransform' input arguments.
  // Initialize function input argument 'camera_pos_V'.
  argInit_3x1_real_T(camera_pos_V_tmp);
  // Initialize function input argument 'camera_dir_V'.
  // Initialize function input argument 'camera_up_V'.
  // Initialize function input argument 'cog_pos_V'.
  // Call the entry-point 'initializeCameraToCoGTransform'.
  initializeCameraToCoGTransform(camera_pos_V_tmp, camera_pos_V_tmp,
                                 camera_pos_V_tmp, camera_pos_V_tmp,
                                 T_body_camera);
}

void main_transformBodyToENU()
{
  double dv[4];
  double p_body_tmp[3];
  double p_enu[3];
  // Initialize function 'transformBodyToENU' input arguments.
  // Initialize function input argument 'p_body'.
  argInit_3x1_real_T(p_body_tmp);
  // Initialize function input argument 'p_cog_enu'.
  // Initialize function input argument 'q'.
  // Call the entry-point 'transformBodyToENU'.
  argInit_4x1_real_T(dv);
  transformBodyToENU(p_body_tmp, p_body_tmp, dv, p_enu);
}

void main_transformCameraToCoG()
{
  double dv1[16];
  double dv[3];
  double p_body[3];
  // Initialize function 'transformCameraToCoG' input arguments.
  // Initialize function input argument 'p_camera'.
  // Initialize function input argument 'T_body_camera'.
  // Call the entry-point 'transformCameraToCoG'.
  argInit_3x1_real_T(dv);
  argInit_4x4_real_T(dv1);
  transformCameraToCoG(dv, dv1, p_body);
}

// End of code generation (main.cpp)
