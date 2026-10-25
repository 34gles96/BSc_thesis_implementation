//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// accelerometer_correction_step.cpp
//
// Code generation for function 'accelerometer_correction_step'
//

// Include files
#include "accelerometer_correction_step.h"
#include "EKF_func_data.h"
#include "EKF_func_types.h"
#include "rt_nonfinite.h"
#include <algorithm>
#include <cmath>
#include <cstring>

// Function Definitions
boolean_T accelerometer_correction_step(const double IMU_accel[3],
                                        const double IMU_accel_covariance[9],
                                        double Vehicle_velocity,
                                        double Vehicle_steering_angle,
                                        double Vehicle_wheelbase,
                                        struct4_T *state,
                                        double b_prev_velocity, double dt)
{
  double P_posterior[100];
  double H[30];
  double K[30];
  double H_q[12];
  double K_q[12];
  double A[9];
  double S[9];
  boolean_T valid;
  //
  //  Gravity-based accelerometer correction.
  //
  //  IMPORTANT:
  //
  //  This correction modifies:
  //
  //    state.q
  //    state.bg
  //
  //  It does NOT modify:
  //
  //    state.p
  //
  //  Covariance is updated consistently using
  //  the Joseph form.
  //
  //  Assumption:
  //  IMU.accel is expressed in BODY coordinates.
  //
  //  State ordering:
  //
  //  x =
  //  [
  //    p(3)
  //    q(4)
  //    bg(3)
  //  ]
  //  =========================================================
  //  1. A PRIORI QUATERNION AND BIAS
  //  =========================================================
  if (std::isinf(dt) || std::isnan(dt) || (dt <= 0.0)) {
    valid = false;
  } else {
    double absxk;
    double q_norm;
    double scale;
    double t;
    scale = 3.3121686421112381E-170;
    absxk = std::abs(state->q[0]);
    if (absxk > 3.3121686421112381E-170) {
      q_norm = 1.0;
      scale = absxk;
    } else {
      t = absxk / 3.3121686421112381E-170;
      q_norm = t * t;
    }
    absxk = std::abs(state->q[1]);
    if (absxk > scale) {
      t = scale / absxk;
      q_norm = q_norm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      q_norm += t * t;
    }
    absxk = std::abs(state->q[2]);
    if (absxk > scale) {
      t = scale / absxk;
      q_norm = q_norm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      q_norm += t * t;
    }
    absxk = std::abs(state->q[3]);
    if (absxk > scale) {
      t = scale / absxk;
      q_norm = q_norm * t * t + 1.0;
      scale = absxk;
    } else {
      t = absxk / scale;
      q_norm += t * t;
    }
    q_norm = scale * std::sqrt(q_norm);
    if (std::isinf(q_norm) || std::isnan(q_norm) || (q_norm < 1.0E-12)) {
      valid = false;
    } else {
      double q[4];
      double q_prior[4];
      double innovation_acc[3];
      double S_tmp;
      double b_S_tmp;
      int r1;
      q_prior[0] = state->q[0] / q_norm;
      q_prior[1] = state->q[1] / q_norm;
      q_prior[2] = state->q[2] / q_norm;
      q_prior[3] = state->q[3] / q_norm;
      //  =========================================================
      //  2. CALCULATE ACCELEROMETER INNOVATION
      //
      //  Also performs low-dynamics validity check.
      //  =========================================================
      //
      //  Calculates gravity-based accelerometer innovation.
      //
      //  All acceleration vectors used for the innovation are
      //  expressed in BODY coordinates.
      //
      //  Assumption:
      //
      //  Vehicle dynamic acceleration is sufficiently small so that
      //  the accelerometer measurement is dominated by gravity.
      //  =========================================================
      //  CONSTANTS
      //  =========================================================
      //  [m/s^2]
      //  =========================================================
      //  DEFAULT OUTPUTS
      //  =========================================================
      innovation_acc[0] = 0.0;
      innovation_acc[1] = 0.0;
      innovation_acc[2] = 0.0;
      valid = false;
      //  =========================================================
      //  BASIC CHECKS
      //  =========================================================
      if (!(Vehicle_wheelbase <= 0.0)) {
        //  =========================================================
        //  1. LONGITUDINAL ACCELERATION
        //
        //                  v(k) - v(k-1)
        //  a_long = -----------------------------
        //                        dt
        //  =========================================================
        q_norm = (Vehicle_velocity - b_prev_velocity) / dt;
        //  =========================================================
        //  2. LATERAL ACCELERATION FROM BICYCLE MODEL
        //
        //  yaw_rate =
        //
        //            v
        //           --- tan(delta)
        //            L
        //
        //  and:
        //
        //  a_lat = v * yaw_rate
        //
        //  therefore:
        //
        //              v^2
        //  a_lat = ----------- tan(delta)
        //               L
        //  =========================================================
        absxk = Vehicle_velocity * Vehicle_velocity / Vehicle_wheelbase *
                std::tan(Vehicle_steering_angle);
        //  =========================================================
        //  3. ESTIMATED DYNAMIC HORIZONTAL ACCELERATION
        //  =========================================================
        //  4. FIRST LOW-DYNAMICS CONDITION
        //  =========================================================
        if (!(std::sqrt(q_norm * q_norm + absxk * absxk) >= 0.5)) {
          //  =========================================================
          //  5. ACCELEROMETER MAGNITUDE CHECK
          //
          //  norm([ax ay az]') should be close to g
          //  if gravity dominates.
          //  =========================================================
          scale = 3.3121686421112381E-170;
          absxk = std::abs(IMU_accel[0]);
          if (absxk > 3.3121686421112381E-170) {
            q_norm = 1.0;
            scale = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            q_norm = t * t;
          }
          absxk = std::abs(IMU_accel[1]);
          if (absxk > scale) {
            t = scale / absxk;
            q_norm = q_norm * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            q_norm += t * t;
          }
          absxk = std::abs(IMU_accel[2]);
          if (absxk > scale) {
            t = scale / absxk;
            q_norm = q_norm * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            q_norm += t * t;
          }
          q_norm = scale * std::sqrt(q_norm);
          if (!(std::abs(q_norm - 9.81) >= 0.5)) {
            //  =========================================================
            //  6. NORMALIZE A PRIORI QUATERNION
            //
            //  q describes BODY -> ENU
            //  =========================================================
            scale = 3.3121686421112381E-170;
            absxk = std::abs(q_prior[0]);
            if (absxk > 3.3121686421112381E-170) {
              q_norm = 1.0;
              scale = absxk;
            } else {
              t = absxk / 3.3121686421112381E-170;
              q_norm = t * t;
            }
            absxk = std::abs(q_prior[1]);
            if (absxk > scale) {
              t = scale / absxk;
              q_norm = q_norm * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              q_norm += t * t;
            }
            absxk = std::abs(q_prior[2]);
            if (absxk > scale) {
              t = scale / absxk;
              q_norm = q_norm * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              q_norm += t * t;
            }
            absxk = std::abs(q_prior[3]);
            if (absxk > scale) {
              t = scale / absxk;
              q_norm = q_norm * t * t + 1.0;
              scale = absxk;
            } else {
              t = absxk / scale;
              q_norm += t * t;
            }
            q_norm = scale * std::sqrt(q_norm);
            q[0] = q_prior[0] / q_norm;
            q[1] = q_prior[1] / q_norm;
            q[2] = q_prior[2] / q_norm;
            q[3] = q_prior[3] / q_norm;
            //  =========================================================
            //  7. BODY -> ENU ROTATION MATRIX
            //  =========================================================
            //  8. ENU -> BODY ROTATION
            //  =========================================================
            //  9. GRAVITY-RELATED SPECIFIC FORCE IN ENU
            //
            //  Actual gravitational acceleration is:
            //
            //      [0]
            //  g = [0]
            //      [-g]
            //
            //  A stationary accelerometer measures opposite gravity:
            //
            //      [0]
            //      [+g]
            //  =========================================================
            //  10. PREDICT ACCELEROMETER MEASUREMENT IN BODY FRAME
            //  =========================================================
            //  11. INNOVATION
            //
            //  measured - predicted
            //
            //  BOTH are in BODY coordinates.
            //  =========================================================
            absxk = q[3] * q[3];
            q_norm = q[2] * q[2];
            S[0] = 1.0 - 2.0 * (q_norm + absxk);
            t = q[1] * q[2];
            scale = q[0] * q[3];
            S[1] = 2.0 * (t - scale);
            S_tmp = q[1] * q[3];
            b_S_tmp = q[0] * q[2];
            S[2] = 2.0 * (S_tmp + b_S_tmp);
            S[3] = 2.0 * (t + scale);
            t = q[1] * q[1];
            S[4] = 1.0 - 2.0 * (t + absxk);
            absxk = q[2] * q[3];
            scale = q[0] * q[1];
            S[5] = 2.0 * (absxk - scale);
            S[6] = 2.0 * (S_tmp - b_S_tmp);
            S[7] = 2.0 * (absxk + scale);
            S[8] = 1.0 - 2.0 * (t + q_norm);
            for (r1 = 0; r1 < 3; r1++) {
              innovation_acc[r1] =
                  IMU_accel[r1] -
                  ((S[r1] * 0.0 + S[r1 + 3] * 0.0) + S[r1 + 6] * 9.81);
            }
            //  =========================================================
            //  MEASUREMENT IS ACCEPTED
            //  =========================================================
            valid = true;
          }
        }
      }
      //  =========================================================
      //  3. IF GRAVITY MODEL IS NOT VALID:
      //     SKIP THIS CORRECTION
      //  =========================================================
      if (valid) {
        int rtemp;
        boolean_T x_data[100];
        boolean_T exitg1;
        boolean_T y;
        x_data[0] =
            (std::isinf(innovation_acc[0]) || std::isnan(innovation_acc[0]));
        x_data[1] =
            (std::isinf(innovation_acc[1]) || std::isnan(innovation_acc[1]));
        x_data[2] =
            (std::isinf(innovation_acc[2]) || std::isnan(innovation_acc[2]));
        y = false;
        rtemp = 0;
        exitg1 = false;
        while ((!exitg1) && (rtemp <= 2)) {
          if (x_data[rtemp]) {
            y = true;
            exitg1 = true;
          } else {
            rtemp++;
          }
        }
        if (y) {
          valid = false;
        } else {
          double y_tmp[12];
          double d;
          double d1;
          int K_q_tmp;
          int b_K_q_tmp;
          //  =========================================================
          //  4. EXTRACT RELEVANT COVARIANCE BLOCKS
          //  =========================================================
          //  5. ACCELEROMETER MEASUREMENT JACOBIAN
          //
          //  H_q:
          //
          //       d(acc_pred_body)
          //       ----------------
          //              dq
          //
          //  Dimension: 3x4
          //
          //  Full H has dimension 3x10:
          //
          //  H = [zeros(3,3), H_q, zeros(3,3)]
          //
          //  Measurement prediction depends directly
          //  only on the quaternion.
          //  =========================================================
          //
          //  Accelerometer measurement Jacobian with respect
          //  to quaternion only.
          //
          //  Quaternion:
          //
          //  q = [qw qx qy qz]'
          //
          //  Predicted accelerometer:
          //
          //  a_pred_body =
          //
          //      R_BE(q) * [0; 0; g]
          //
          //  H_q dimension:
          //
          //      3 x 4
          //
          //  because:
          //
          //  3 accelerometer measurements
          //  4 quaternion state components
          //  =========================================================
          //  NORMALIZE QUATERNION
          //  =========================================================
          scale = 3.3121686421112381E-170;
          absxk = std::abs(q_prior[0]);
          if (absxk > 3.3121686421112381E-170) {
            q_norm = 1.0;
            scale = absxk;
          } else {
            t = absxk / 3.3121686421112381E-170;
            q_norm = t * t;
          }
          absxk = std::abs(q_prior[1]);
          if (absxk > scale) {
            t = scale / absxk;
            q_norm = q_norm * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            q_norm += t * t;
          }
          absxk = std::abs(q_prior[2]);
          if (absxk > scale) {
            t = scale / absxk;
            q_norm = q_norm * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            q_norm += t * t;
          }
          absxk = std::abs(q_prior[3]);
          if (absxk > scale) {
            t = scale / absxk;
            q_norm = q_norm * t * t + 1.0;
            scale = absxk;
          } else {
            t = absxk / scale;
            q_norm += t * t;
          }
          q_norm = scale * std::sqrt(q_norm);
          q[0] = q_prior[0] / q_norm;
          q[1] = q_prior[1] / q_norm;
          q[2] = q_prior[2] / q_norm;
          //  =========================================================
          //  Predicted acceleration:
          //
          //  ax = 2g(qx*qz - qw*qy)
          //
          //  ay = 2g(qy*qz + qw*qx)
          //
          //  az = g(1 - 2*qx^2 - 2*qy^2)
          //
          //  H_q =
          //
          //       d[a_x a_y a_z]'
          //       ----------------
          //       d[qw qx qy qz]
          //  =========================================================
          H_q[0] = 9.81 * (-2.0 * q[2]);
          q_norm = 9.81 * (2.0 * (q_prior[3] / q_norm));
          H_q[3] = q_norm;
          H_q[6] = 9.81 * (-2.0 * q[0]);
          absxk = 9.81 * (2.0 * q[1]);
          H_q[9] = absxk;
          H_q[1] = absxk;
          H_q[4] = 9.81 * (2.0 * q[0]);
          H_q[7] = q_norm;
          H_q[10] = 9.81 * (2.0 * q[2]);
          H_q[2] = 0.0;
          H_q[5] = 9.81 * (-4.0 * q[1]);
          H_q[8] = 9.81 * (-4.0 * q[2]);
          H_q[11] = 0.0;
          std::memset(&H[0], 0, 30U * sizeof(double));
          for (r1 = 0; r1 < 4; r1++) {
            rtemp = 3 * (r1 + 3);
            H[rtemp] = H_q[3 * r1];
            H[rtemp + 1] = H_q[3 * r1 + 1];
            H[rtemp + 2] = H_q[3 * r1 + 2];
          }
          //  =========================================================
          //  6. ACCELEROMETER MEASUREMENT COVARIANCE
          //
          //  If IMU.accel_covariance already contains VARIANCES,
          //  DO NOT square them again.
          //  =========================================================
          //  7. INNOVATION COVARIANCE
          //
          //  S = Hq Pqq Hq' + Racc
          //
          //  Dimension: 3x3
          //  =========================================================
          for (r1 = 0; r1 < 3; r1++) {
            for (K_q_tmp = 0; K_q_tmp < 4; K_q_tmp++) {
              rtemp = r1 + 3 * K_q_tmp;
              y_tmp[K_q_tmp + (r1 << 2)] = H_q[rtemp];
              b_K_q_tmp = 10 * (K_q_tmp + 3);
              K_q[rtemp] = ((H_q[r1] * state->P[b_K_q_tmp + 3] +
                             H_q[r1 + 3] * state->P[b_K_q_tmp + 4]) +
                            H_q[r1 + 6] * state->P[b_K_q_tmp + 5]) +
                           H_q[r1 + 9] * state->P[b_K_q_tmp + 6];
            }
          }
          for (r1 = 0; r1 < 3; r1++) {
            d = K_q[r1];
            b_S_tmp = K_q[r1 + 3];
            d1 = K_q[r1 + 6];
            S_tmp = K_q[r1 + 9];
            for (K_q_tmp = 0; K_q_tmp < 3; K_q_tmp++) {
              b_K_q_tmp = K_q_tmp << 2;
              rtemp = r1 + 3 * K_q_tmp;
              S[rtemp] =
                  (((d * y_tmp[b_K_q_tmp] + b_S_tmp * y_tmp[b_K_q_tmp + 1]) +
                    d1 * y_tmp[b_K_q_tmp + 2]) +
                   S_tmp * y_tmp[b_K_q_tmp + 3]) +
                  IMU_accel_covariance[rtemp];
            }
          }
          for (r1 = 0; r1 < 9; r1++) {
            d = S[r1];
            x_data[r1] = (std::isinf(d) || std::isnan(d));
          }
          y = false;
          rtemp = 0;
          exitg1 = false;
          while ((!exitg1) && (rtemp <= 8)) {
            if (x_data[rtemp]) {
              y = true;
              exitg1 = true;
            } else {
              rtemp++;
            }
          }
          if (y) {
            valid = false;
          } else {
            double K_b[9];
            double a21_tmp;
            double maxval;
            int r2;
            int r3;
            //  =========================================================
            //  8. KALMAN GAIN
            //
            //  Quaternion gain:
            //
            //  Kq = Pqq Hq' / S
            //
            //  Dimension: 4x3
            //
            //  Bias gain:
            //
            //  Kb = Pbq Hq' / S
            //
            //  Dimension: 3x3
            //
            //  Position gain is deliberately zero.
            //  =========================================================
            for (r1 = 0; r1 < 4; r1++) {
              d = state->P[r1 + 33];
              b_S_tmp = state->P[r1 + 43];
              d1 = state->P[r1 + 53];
              S_tmp = state->P[r1 + 63];
              for (K_q_tmp = 0; K_q_tmp < 3; K_q_tmp++) {
                b_K_q_tmp = K_q_tmp << 2;
                K_q[r1 + b_K_q_tmp] =
                    ((d * y_tmp[b_K_q_tmp] + b_S_tmp * y_tmp[b_K_q_tmp + 1]) +
                     d1 * y_tmp[b_K_q_tmp + 2]) +
                    S_tmp * y_tmp[b_K_q_tmp + 3];
              }
            }
            std::copy(&K_q[0], &K_q[12], &H_q[0]);
            std::copy(&S[0], &S[9], &A[0]);
            r1 = 0;
            r2 = 1;
            r3 = 2;
            maxval = std::abs(S[0]);
            q_norm = maxval;
            a21_tmp = std::abs(S[1]);
            if (a21_tmp > maxval) {
              q_norm = a21_tmp;
              r1 = 1;
              r2 = 0;
            }
            d = std::abs(S[2]);
            if (d > q_norm) {
              r1 = 2;
              r2 = 1;
              r3 = 0;
            }
            A[r2] = S[r2] / S[r1];
            A[r3] /= A[r1];
            A[r2 + 3] -= A[r2] * A[r1 + 3];
            A[r3 + 3] -= A[r3] * A[r1 + 3];
            A[r2 + 6] -= A[r2] * A[r1 + 6];
            A[r3 + 6] -= A[r3] * A[r1 + 6];
            if (std::abs(A[r3 + 3]) > std::abs(A[r2 + 3])) {
              rtemp = r2;
              r2 = r3;
              r3 = rtemp;
            }
            A[r3 + 3] /= A[r2 + 3];
            A[r3 + 6] -= A[r3 + 3] * A[r2 + 6];
            rtemp = r1 << 2;
            K_q[rtemp] = H_q[0] / A[r1];
            b_K_q_tmp = r2 << 2;
            S_tmp = A[r1 + 3];
            K_q[b_K_q_tmp] = H_q[4] - K_q[rtemp] * S_tmp;
            K_q_tmp = r3 << 2;
            b_S_tmp = A[r1 + 6];
            K_q[K_q_tmp] = H_q[8] - K_q[rtemp] * b_S_tmp;
            absxk = A[r2 + 3];
            K_q[b_K_q_tmp] /= absxk;
            t = A[r2 + 6];
            K_q[K_q_tmp] -= K_q[b_K_q_tmp] * t;
            q_norm = A[r3 + 6];
            K_q[K_q_tmp] /= q_norm;
            scale = A[r3 + 3];
            K_q[b_K_q_tmp] -= K_q[K_q_tmp] * scale;
            K_q[rtemp] -= K_q[K_q_tmp] * A[r3];
            K_q[rtemp] -= K_q[b_K_q_tmp] * A[r2];
            K_q[rtemp + 1] = H_q[1] / A[r1];
            K_q[b_K_q_tmp + 1] = H_q[5] - K_q[rtemp + 1] * S_tmp;
            K_q[K_q_tmp + 1] = H_q[9] - K_q[rtemp + 1] * b_S_tmp;
            K_q[b_K_q_tmp + 1] /= absxk;
            K_q[K_q_tmp + 1] -= K_q[b_K_q_tmp + 1] * t;
            K_q[K_q_tmp + 1] /= q_norm;
            K_q[b_K_q_tmp + 1] -= K_q[K_q_tmp + 1] * scale;
            K_q[rtemp + 1] -= K_q[K_q_tmp + 1] * A[r3];
            K_q[rtemp + 1] -= K_q[b_K_q_tmp + 1] * A[r2];
            K_q[rtemp + 2] = H_q[2] / A[r1];
            K_q[b_K_q_tmp + 2] = H_q[6] - K_q[rtemp + 2] * S_tmp;
            K_q[K_q_tmp + 2] = H_q[10] - K_q[rtemp + 2] * b_S_tmp;
            K_q[b_K_q_tmp + 2] /= absxk;
            K_q[K_q_tmp + 2] -= K_q[b_K_q_tmp + 2] * t;
            K_q[K_q_tmp + 2] /= q_norm;
            K_q[b_K_q_tmp + 2] -= K_q[K_q_tmp + 2] * scale;
            K_q[rtemp + 2] -= K_q[K_q_tmp + 2] * A[r3];
            K_q[rtemp + 2] -= K_q[b_K_q_tmp + 2] * A[r2];
            K_q[rtemp + 3] = H_q[3] / A[r1];
            K_q[b_K_q_tmp + 3] = H_q[7] - K_q[rtemp + 3] * S_tmp;
            K_q[K_q_tmp + 3] = H_q[11] - K_q[rtemp + 3] * b_S_tmp;
            K_q[b_K_q_tmp + 3] /= absxk;
            K_q[K_q_tmp + 3] -= K_q[b_K_q_tmp + 3] * t;
            K_q[K_q_tmp + 3] /= q_norm;
            K_q[b_K_q_tmp + 3] -= K_q[K_q_tmp + 3] * scale;
            K_q[rtemp + 3] -= K_q[K_q_tmp + 3] * A[r3];
            K_q[rtemp + 3] -= K_q[b_K_q_tmp + 3] * A[r2];
            for (r1 = 0; r1 < 3; r1++) {
              b_S_tmp = state->P[r1 + 37];
              d1 = state->P[r1 + 47];
              S_tmp = state->P[r1 + 57];
              absxk = state->P[r1 + 67];
              for (K_q_tmp = 0; K_q_tmp < 3; K_q_tmp++) {
                b_K_q_tmp = K_q_tmp << 2;
                A[r1 + 3 * K_q_tmp] =
                    ((b_S_tmp * y_tmp[b_K_q_tmp] + d1 * y_tmp[b_K_q_tmp + 1]) +
                     S_tmp * y_tmp[b_K_q_tmp + 2]) +
                    absxk * y_tmp[b_K_q_tmp + 3];
              }
            }
            r1 = 0;
            r2 = 1;
            r3 = 2;
            if (a21_tmp > maxval) {
              maxval = a21_tmp;
              r1 = 1;
              r2 = 0;
            }
            if (d > maxval) {
              r1 = 2;
              r2 = 1;
              r3 = 0;
            }
            S[r2] /= S[r1];
            S[r3] /= S[r1];
            S[r2 + 3] -= S[r2] * S[r1 + 3];
            S[r3 + 3] -= S[r3] * S[r1 + 3];
            S[r2 + 6] -= S[r2] * S[r1 + 6];
            S[r3 + 6] -= S[r3] * S[r1 + 6];
            if (std::abs(S[r3 + 3]) > std::abs(S[r2 + 3])) {
              rtemp = r2;
              r2 = r3;
              r3 = rtemp;
            }
            S[r3 + 3] /= S[r2 + 3];
            S[r3 + 6] -= S[r3 + 3] * S[r2 + 6];
            K_b[3 * r1] = A[0] / S[r1];
            absxk = S[r1 + 3];
            K_b[3 * r2] = A[3] - K_b[3 * r1] * absxk;
            q_norm = S[r1 + 6];
            K_b[3 * r3] = A[6] - K_b[3 * r1] * q_norm;
            t = S[r2 + 3];
            K_b[3 * r2] /= t;
            scale = S[r2 + 6];
            K_b[3 * r3] -= K_b[3 * r2] * scale;
            S_tmp = S[r3 + 6];
            K_b[3 * r3] /= S_tmp;
            b_S_tmp = S[r3 + 3];
            K_b[3 * r2] -= K_b[3 * r3] * b_S_tmp;
            K_b[3 * r1] -= K_b[3 * r3] * S[r3];
            K_b[3 * r1] -= K_b[3 * r2] * S[r2];
            rtemp = 3 * r1 + 1;
            K_b[rtemp] = A[1] / S[r1];
            b_K_q_tmp = 3 * r2 + 1;
            K_b[b_K_q_tmp] = A[4] - K_b[rtemp] * absxk;
            K_q_tmp = 3 * r3 + 1;
            K_b[K_q_tmp] = A[7] - K_b[rtemp] * q_norm;
            K_b[b_K_q_tmp] /= t;
            K_b[K_q_tmp] -= K_b[b_K_q_tmp] * scale;
            K_b[K_q_tmp] /= S_tmp;
            K_b[b_K_q_tmp] -= K_b[K_q_tmp] * b_S_tmp;
            K_b[rtemp] -= K_b[K_q_tmp] * S[r3];
            K_b[rtemp] -= K_b[b_K_q_tmp] * S[r2];
            rtemp = 3 * r1 + 2;
            K_b[rtemp] = A[2] / S[r1];
            b_K_q_tmp = 3 * r2 + 2;
            K_b[b_K_q_tmp] = A[5] - K_b[rtemp] * absxk;
            K_q_tmp = 3 * r3 + 2;
            K_b[K_q_tmp] = A[8] - K_b[rtemp] * q_norm;
            K_b[b_K_q_tmp] /= t;
            K_b[K_q_tmp] -= K_b[b_K_q_tmp] * scale;
            K_b[K_q_tmp] /= S_tmp;
            K_b[b_K_q_tmp] -= K_b[K_q_tmp] * b_S_tmp;
            K_b[rtemp] -= K_b[K_q_tmp] * S[r3];
            K_b[rtemp] -= K_b[b_K_q_tmp] * S[r2];
            //  Complete gain for the 10-state covariance update
            std::memset(&K[0], 0, 30U * sizeof(double));
            for (r1 = 0; r1 < 3; r1++) {
              rtemp = r1 << 2;
              K[10 * r1 + 3] = K_q[rtemp];
              K[10 * r1 + 4] = K_q[rtemp + 1];
              K[10 * r1 + 5] = K_q[rtemp + 2];
              K[10 * r1 + 6] = K_q[rtemp + 3];
              K[10 * r1 + 7] = K_b[3 * r1];
              K[10 * r1 + 8] = K_b[3 * r1 + 1];
              K[10 * r1 + 9] = K_b[3 * r1 + 2];
            }
            for (r1 = 0; r1 < 30; r1++) {
              d = K[r1];
              x_data[r1] = (std::isinf(d) || std::isnan(d));
            }
            y = false;
            rtemp = 0;
            exitg1 = false;
            while ((!exitg1) && (rtemp <= 29)) {
              if (x_data[rtemp]) {
                y = true;
                exitg1 = true;
              } else {
                rtemp++;
              }
            }
            if (y) {
              valid = false;
            } else {
              //  =========================================================
              //  9. QUATERNION AND GYRO BIAS CORRECTION
              //  =========================================================
              //  =========================================================
              //  10. NORMALIZE QUATERNION
              //  =========================================================
              q_norm = 0.0;
              scale = 3.3121686421112381E-170;
              d = innovation_acc[0];
              b_S_tmp = innovation_acc[1];
              d1 = innovation_acc[2];
              for (rtemp = 0; rtemp < 4; rtemp++) {
                S_tmp = q_prior[rtemp] +
                        ((K_q[rtemp] * d + K_q[rtemp + 4] * b_S_tmp) +
                         K_q[rtemp + 8] * d1);
                q[rtemp] = S_tmp;
                absxk = std::abs(S_tmp);
                if (absxk > scale) {
                  t = scale / absxk;
                  q_norm = q_norm * t * t + 1.0;
                  scale = absxk;
                } else {
                  t = absxk / scale;
                  q_norm += t * t;
                }
              }
              q_norm = scale * std::sqrt(q_norm);
              if (std::isinf(q_norm) || std::isnan(q_norm) ||
                  (q_norm < 1.0E-12)) {
                valid = false;
              } else {
                double b_A[100];
                double c_A[100];
                double b_K[30];
                q[0] /= q_norm;
                q[1] /= q_norm;
                q[2] /= q_norm;
                q[3] /= q_norm;
                //  =========================================================
                //  11. JOSEPH-FORM COVARIANCE UPDATE
                //
                //  P+ = (I-KH) P- (I-KH)' + K R K'
                //
                //  IMPORTANT:
                //
                //  The position gain K(1:3,:) is zero.
                //
                //  Therefore the position state is not corrected.
                //
                //  However, covariance cross-blocks involving position
                //  are still updated to preserve consistency.
                //
                //  This also updates:
                //
                //    Pqq
                //    Pqb
                //    Pbq
                //    Pbb
                //  =========================================================
                for (r1 = 0; r1 < 10; r1++) {
                  d = K[r1];
                  b_S_tmp = K[r1 + 10];
                  d1 = K[r1 + 20];
                  for (K_q_tmp = 0; K_q_tmp < 10; K_q_tmp++) {
                    rtemp = r1 + 10 * K_q_tmp;
                    b_A[rtemp] =
                        static_cast<double>(iv[rtemp]) -
                        ((d * H[3 * K_q_tmp] + b_S_tmp * H[3 * K_q_tmp + 1]) +
                         d1 * H[3 * K_q_tmp + 2]);
                  }
                  for (K_q_tmp = 0; K_q_tmp < 10; K_q_tmp++) {
                    S_tmp = 0.0;
                    for (b_K_q_tmp = 0; b_K_q_tmp < 10; b_K_q_tmp++) {
                      S_tmp += b_A[r1 + 10 * b_K_q_tmp] *
                               state->P[b_K_q_tmp + 10 * K_q_tmp];
                    }
                    c_A[r1 + 10 * K_q_tmp] = S_tmp;
                  }
                  for (K_q_tmp = 0; K_q_tmp < 3; K_q_tmp++) {
                    b_K[r1 + 10 * K_q_tmp] =
                        (d * IMU_accel_covariance[3 * K_q_tmp] +
                         b_S_tmp * IMU_accel_covariance[3 * K_q_tmp + 1]) +
                        d1 * IMU_accel_covariance[3 * K_q_tmp + 2];
                  }
                }
                for (r1 = 0; r1 < 10; r1++) {
                  for (K_q_tmp = 0; K_q_tmp < 10; K_q_tmp++) {
                    d = 0.0;
                    for (b_K_q_tmp = 0; b_K_q_tmp < 10; b_K_q_tmp++) {
                      d += c_A[r1 + 10 * b_K_q_tmp] *
                           b_A[K_q_tmp + 10 * b_K_q_tmp];
                    }
                    P_posterior[r1 + 10 * K_q_tmp] = d;
                  }
                }
                for (r1 = 0; r1 < 10; r1++) {
                  d = b_K[r1];
                  b_S_tmp = b_K[r1 + 10];
                  d1 = b_K[r1 + 20];
                  for (K_q_tmp = 0; K_q_tmp < 10; K_q_tmp++) {
                    b_A[r1 + 10 * K_q_tmp] =
                        (d * K[K_q_tmp] + b_S_tmp * K[K_q_tmp + 10]) +
                        d1 * K[K_q_tmp + 20];
                  }
                }
                for (r1 = 0; r1 < 100; r1++) {
                  P_posterior[r1] += b_A[r1];
                }
                //  Optional numerical symmetry enforcement
                for (r1 = 0; r1 < 10; r1++) {
                  for (K_q_tmp = 0; K_q_tmp < 10; K_q_tmp++) {
                    rtemp = K_q_tmp + 10 * r1;
                    b_A[rtemp] = 0.5 * (P_posterior[rtemp] +
                                        P_posterior[r1 + 10 * K_q_tmp]);
                  }
                }
                for (r1 = 0; r1 < 100; r1++) {
                  d = b_A[r1];
                  P_posterior[r1] = d;
                  x_data[r1] = (std::isinf(d) || std::isnan(d));
                }
                y = false;
                rtemp = 0;
                exitg1 = false;
                while ((!exitg1) && (rtemp <= 99)) {
                  if (x_data[rtemp]) {
                    y = true;
                    exitg1 = true;
                  } else {
                    rtemp++;
                  }
                }
                if (y) {
                  valid = false;
                } else {
                  //  =========================================================
                  //  12. STORE POSTERIOR STATE
                  //  =========================================================
                  //  Position is intentionally unchanged.
                  state->q[0] = q[0];
                  state->q[1] = q[1];
                  state->q[2] = q[2];
                  state->q[3] = q[3];
                  d = innovation_acc[0];
                  b_S_tmp = innovation_acc[1];
                  d1 = innovation_acc[2];
                  for (r1 = 0; r1 < 3; r1++) {
                    state->bg[r1] += (K_b[r1] * d + K_b[r1 + 3] * b_S_tmp) +
                                     K_b[r1 + 6] * d1;
                  }
                  std::copy(&P_posterior[0], &P_posterior[100], &state->P[0]);
                }
              }
            }
          }
        }
      }
    }
  }
  return valid;
}

// End of code generation (accelerometer_correction_step.cpp)
