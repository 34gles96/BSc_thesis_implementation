//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// mpcController.cpp
//
// Code generation for function 'mpcController'
//

// Include files
#include "mpcController.h"
#include "anonymous_function.h"
#include "fmincon.h"
#include "mpcController_internal_types1.h"
#include "mpcController_types.h"
#include "rt_nonfinite.h"
#include "coder_array.h"
#include "rt_defines.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Declarations
static double rt_atan2d_snf(double u0, double u1);

// Function Definitions
static double rt_atan2d_snf(double u0, double u1)
{
  double y;
  if (std::isnan(u0) || std::isnan(u1)) {
    y = rtNaN;
  } else if (std::isinf(u0) && std::isinf(u1)) {
    int i;
    int i1;
    if (u0 > 0.0) {
      i = 1;
    } else {
      i = -1;
    }
    if (u1 > 0.0) {
      i1 = 1;
    } else {
      i1 = -1;
    }
    y = std::atan2(static_cast<double>(i), static_cast<double>(i1));
  } else if (u1 == 0.0) {
    if (u0 > 0.0) {
      y = RT_PI / 2.0;
    } else if (u0 < 0.0) {
      y = -(RT_PI / 2.0);
    } else {
      y = 0.0;
    }
  } else {
    y = std::atan2(u0, u1);
  }
  return y;
}

void mpcController(const double x_current[4], const double refHorizon[40],
                   const double u_last_applied[2], const struct0_T *params,
                   double u_optimal[2], coder::array<double, 2U> &U_optimal)
{
  coder::anonymous_function costFunction;
  coder::array<double, 1U> U_initial;
  coder::array<double, 1U> lowerBound;
  coder::array<double, 1U> upperBound;
  int loop_ub;
  int ntilerows_tmp;
  //  MPC parameters
  costFunction.workspace.Ts = params->Ts;
  costFunction.workspace.L = params->L;
  std::copy(&params->Q[0], &params->Q[16], &costFunction.workspace.Q[0]);
  //  State tracking error
  //  Control effort
  costFunction.workspace.R[0] = params->R[0];
  costFunction.workspace.S[0] = params->S[0];
  costFunction.workspace.R[1] = params->R[1];
  costFunction.workspace.S[1] = params->S[1];
  costFunction.workspace.R[2] = params->R[2];
  costFunction.workspace.S[2] = params->S[2];
  costFunction.workspace.R[3] = params->R[3];
  costFunction.workspace.S[3] = params->S[3];
  //  Control change
  //  Initial guess for fmincon:
  //  use the last applied control over the whole prediction horizon
  loop_ub = static_cast<int>(params->Np) << 1;
  U_initial.set_size(loop_ub);
  ntilerows_tmp = static_cast<int>(params->Np);
  //  Control limits
  lowerBound.set_size(loop_ub);
  upperBound.set_size(loop_ub);
  for (int itilerow{0}; itilerow < ntilerows_tmp; itilerow++) {
    int ibcol_tmp;
    ibcol_tmp = itilerow << 1;
    U_initial[ibcol_tmp] = u_last_applied[0];
    U_initial[ibcol_tmp + 1] = u_last_applied[1];
    lowerBound[ibcol_tmp] = -params->deltaMax;
    lowerBound[ibcol_tmp + 1] = params->aMin;
    upperBound[ibcol_tmp] = params->deltaMax;
    upperBound[ibcol_tmp + 1] = params->aMax;
  }
  //  @(U_vector) declares U_vector as the input of this function.
  //  fmincon supplies and changes U_vector during optimization.
  costFunction.workspace.Np = params->Np;
  costFunction.workspace.x_current[0] = x_current[0];
  costFunction.workspace.x_current[1] = x_current[1];
  costFunction.workspace.x_current[2] = x_current[2];
  costFunction.workspace.x_current[3] = x_current[3];
  std::copy(&refHorizon[0], &refHorizon[40],
            &costFunction.workspace.refHorizon[0]);
  costFunction.workspace.u_last_applied[0] = u_last_applied[0];
  costFunction.workspace.u_last_applied[1] = u_last_applied[1];
  //   options = optimoptions( ...
  //      'fmincon', ...
  //     'Algorithm', 'sqp', ...
  //    'Display', 'none');
  //  Find the optimal future control sequence
  coder::fmincon(costFunction, U_initial, lowerBound, upperBound);
  U_optimal.set_size(2, static_cast<int>(params->Np));
  for (ntilerows_tmp = 0; ntilerows_tmp < loop_ub; ntilerows_tmp++) {
    U_optimal[ntilerows_tmp] = U_initial[ntilerows_tmp];
  }
  //  Receding horizon:
  //  apply only the first optimized control
  u_optimal[0] = U_initial[0];
  u_optimal[1] = U_initial[1];
}

double mpcController_anonFcn1(const double x_current[4],
                              const double refHorizon[40],
                              const double u_last_applied[2], double Np,
                              double Ts, double L, const double Q[16],
                              const double R[4], const double S[4],
                              const coder::array<double, 1U> &U_vector)
{
  double u_previous_idx_0;
  double u_previous_idx_1;
  double varargout_1;
  double x_predicted_idx_0;
  double x_predicted_idx_1;
  double x_predicted_idx_2;
  double x_predicted_idx_3;
  int i;
  //  Control sequence currently evaluated by fmincon
  //  Start prediction from current estimated state
  x_predicted_idx_0 = x_current[0];
  x_predicted_idx_1 = x_current[1];
  x_predicted_idx_2 = x_current[2];
  x_predicted_idx_3 = x_current[3];
  //  Previous real control
  u_previous_idx_0 = u_last_applied[0];
  u_previous_idx_1 = u_last_applied[1];
  varargout_1 = 0.0;
  i = static_cast<int>(Np);
  for (int k{0}; k < i; k++) {
    double b_error[4];
    double c_error;
    int error_tmp;
    //  Control at prediction step k
    //  u = [steering angle; acceleration]
    //  Predict next vehicle state
    //  State:   x = [X; Y; psi; v]
    //  Control: u = [delta; a]
    //  Continuous-time kinematic bicycle model
    //  Euler discretization
    x_predicted_idx_0 += Ts * (x_predicted_idx_3 * std::cos(x_predicted_idx_2));
    x_predicted_idx_1 += Ts * (x_predicted_idx_3 * std::sin(x_predicted_idx_2));
    x_predicted_idx_2 +=
        Ts * (x_predicted_idx_3 / L * std::tan(U_vector[2 * k]));
    x_predicted_idx_3 += Ts * U_vector[2 * k + 1];
    //  Wrap heading to [-pi, pi]
    x_predicted_idx_2 =
        rt_atan2d_snf(std::sin(x_predicted_idx_2), std::cos(x_predicted_idx_2));
    //  Desired state at this future step
    //  Tracking error
    error_tmp = k << 2;
    b_error[0] = x_predicted_idx_0 - refHorizon[error_tmp];
    b_error[1] = x_predicted_idx_1 - refHorizon[error_tmp + 1];
    b_error[2] = x_predicted_idx_2 - refHorizon[error_tmp + 2];
    b_error[3] = x_predicted_idx_3 - refHorizon[error_tmp + 3];
    //  Wrap heading error to [-pi, pi]
    b_error[2] = rt_atan2d_snf(std::sin(b_error[2]), std::cos(b_error[2]));
    //  Change in control
    u_previous_idx_0 = U_vector[2 * k] - u_previous_idx_0;
    u_previous_idx_1 = U_vector[2 * k + 1] - u_previous_idx_1;
    //  Cost
    c_error = 0.0;
    for (int i1{0}; i1 < 4; i1++) {
      error_tmp = i1 << 2;
      c_error += (((b_error[0] * Q[error_tmp] + b_error[1] * Q[error_tmp + 1]) +
                   b_error[2] * Q[error_tmp + 2]) +
                  b_error[3] * Q[error_tmp + 3]) *
                 b_error[i1];
    }
    varargout_1 = ((varargout_1 + c_error) +
                   ((U_vector[2 * k] * R[0] + U_vector[2 * k + 1] * R[1]) *
                        U_vector[2 * k] +
                    (U_vector[2 * k] * R[2] + U_vector[2 * k + 1] * R[3]) *
                        U_vector[2 * k + 1])) +
                  ((u_previous_idx_0 * S[0] + u_previous_idx_1 * S[1]) *
                       u_previous_idx_0 +
                   (u_previous_idx_0 * S[2] + u_previous_idx_1 * S[3]) *
                       u_previous_idx_1);
    //  For the next predicted step
    u_previous_idx_0 = U_vector[2 * k];
    u_previous_idx_1 = U_vector[2 * k + 1];
  }
  return varargout_1;
}

// End of code generation (mpcController.cpp)
