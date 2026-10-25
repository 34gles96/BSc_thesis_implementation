//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_EKF_func_api.cpp
//
// Code generation for function 'EKF_func'
//

// Include files
#include "_coder_EKF_func_api.h"
#include "_coder_EKF_func_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "EKF_func",                                           // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9]);

static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId);

static struct3_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId);

static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4]);

static boolean_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId);

static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[100]);

static real_T e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId);

static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3]);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct2_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y);

static boolean_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId);

static struct3_T emlrt_marshallIn(const emlrtStack &sp,
                                  const mxArray *b_nullptr,
                                  const char_T *identifier);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct4_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct4_T &y);

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3]);

static const mxArray *emlrt_marshallOut(const struct4_T &u);

static const mxArray *emlrt_marshallOut(const real_T u[100]);

static void f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9]);

static void g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4]);

static void h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId,
                               real_T ret[100]);

// Function Definitions
static void b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[9])
{
  f_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId)
{
  real_T y;
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static struct3_T c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                    const emlrtMsgIdentifier *parentId)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[3]{"velocity", "steering_angle", "wheelbase"};
  emlrtMsgIdentifier thisId;
  struct3_T y;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 3,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "velocity";
  y.velocity = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "velocity")),
      &thisId);
  thisId.fIdentifier = "steering_angle";
  y.steering_angle =
      b_emlrt_marshallIn(sp,
                         emlrtAlias(emlrtGetFieldR2017b(
                             (emlrtConstCTX)&sp, u, 0, 1, "steering_angle")),
                         &thisId);
  thisId.fIdentifier = "wheelbase";
  y.wheelbase = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "wheelbase")),
      &thisId);
  emlrtDestroyArray(&u);
  return y;
}

static void c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId, real_T y[4])
{
  g_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static boolean_T d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                    const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  boolean_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "logical", false, 0U,
                          (const void *)&dims);
  ret = *emlrtMxGetLogicals(src);
  emlrtDestroyArray(&src);
  return ret;
}

static void d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                               const emlrtMsgIdentifier *parentId,
                               real_T y[100])
{
  h_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static real_T e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                 const emlrtMsgIdentifier *msgId)
{
  static const int32_T dims{0};
  real_T ret;
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 0U,
                          (const void *)&dims);
  ret = *static_cast<real_T *>(emlrtMxGetData(src));
  emlrtDestroyArray(&src);
  return ret;
}

static void e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[3])
{
  static const int32_T dims{3};
  real_T(*r)[3];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[3])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  emlrtDestroyArray(&src);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct0_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct0_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[7]{
      "valid", "timestamp",        "gyro",    "gyro_covariance",
      "accel", "accel_covariance", "mounting"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 7,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "valid";
  y.valid = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "valid")),
      &thisId);
  thisId.fIdentifier = "timestamp";
  y.timestamp = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "timestamp")),
      &thisId);
  thisId.fIdentifier = "gyro";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "gyro")),
      &thisId, y.gyro);
  thisId.fIdentifier = "gyro_covariance";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3,
                                                    "gyro_covariance")),
                     &thisId, y.gyro_covariance);
  thisId.fIdentifier = "accel";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "accel")),
      &thisId, y.accel);
  thisId.fIdentifier = "accel_covariance";
  b_emlrt_marshallIn(sp,
                     emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                                    "accel_covariance")),
                     &thisId, y.accel_covariance);
  thisId.fIdentifier = "mounting";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6, "mounting")),
      &thisId, y.mounting);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct1_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[3]{"x_sensor_body", "y_sensor_body",
                                     "z_sensor_body"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 3,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "x_sensor_body";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0,
                                                  "x_sensor_body")),
                   &thisId, y.x_sensor_body);
  thisId.fIdentifier = "y_sensor_body";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1,
                                                  "y_sensor_body")),
                   &thisId, y.y_sensor_body);
  thisId.fIdentifier = "z_sensor_body";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2,
                                                  "z_sensor_body")),
                   &thisId, y.z_sensor_body);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct2_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct2_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[13]{
      "valid",         "timestamp",     "lla1",         "lla2",
      "position1_enu", "position2_enu", "position_enu", "baseline_enu",
      "baseline_body", "hAcc1",         "vAcc1",        "hAcc2",
      "vAcc2"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 13,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "valid";
  y.valid = emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "valid")),
      &thisId);
  thisId.fIdentifier = "timestamp";
  y.timestamp = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "timestamp")),
      &thisId);
  thisId.fIdentifier = "lla1";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "lla1")),
      &thisId, y.lla1);
  thisId.fIdentifier = "lla2";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "lla2")),
      &thisId, y.lla2);
  thisId.fIdentifier = "position1_enu";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4,
                                                  "position1_enu")),
                   &thisId, y.position1_enu);
  thisId.fIdentifier = "position2_enu";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5,
                                                  "position2_enu")),
                   &thisId, y.position2_enu);
  thisId.fIdentifier = "position_enu";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 6,
                                                  "position_enu")),
                   &thisId, y.position_enu);
  thisId.fIdentifier = "baseline_enu";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 7,
                                                  "baseline_enu")),
                   &thisId, y.baseline_enu);
  thisId.fIdentifier = "baseline_body";
  emlrt_marshallIn(sp,
                   emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 8,
                                                  "baseline_body")),
                   &thisId, y.baseline_body);
  thisId.fIdentifier = "hAcc1";
  y.hAcc1 = b_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 9, "hAcc1")),
      &thisId);
  thisId.fIdentifier = "vAcc1";
  y.vAcc1 = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 10, "vAcc1")),
      &thisId);
  thisId.fIdentifier = "hAcc2";
  y.hAcc2 = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 11, "hAcc2")),
      &thisId);
  thisId.fIdentifier = "vAcc2";
  y.vAcc2 = b_emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 12, "vAcc2")),
      &thisId);
  emlrtDestroyArray(&u);
}

static boolean_T emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                  const emlrtMsgIdentifier *parentId)
{
  boolean_T y;
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static struct3_T emlrt_marshallIn(const emlrtStack &sp,
                                  const mxArray *b_nullptr,
                                  const char_T *identifier)
{
  emlrtMsgIdentifier thisId;
  struct3_T y;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                             const char_T *identifier, struct4_T &y)
{
  emlrtMsgIdentifier thisId;
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId, y);
  emlrtDestroyArray(&b_nullptr);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, struct4_T &y)
{
  static const int32_T dims{0};
  static const char_T *fieldNames[6]{"p", "q", "bg", "P", "Q", "p_ecef0"};
  emlrtMsgIdentifier thisId;
  thisId.fParent = parentId;
  thisId.bParentIsCell = false;
  emlrtCheckStructR2012b((emlrtConstCTX)&sp, parentId, u, 6,
                         (const char_T **)&fieldNames[0], 0U,
                         (const void *)&dims);
  thisId.fIdentifier = "p";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 0, "p")),
      &thisId, y.p);
  thisId.fIdentifier = "q";
  c_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 1, "q")),
      &thisId, y.q);
  thisId.fIdentifier = "bg";
  emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 2, "bg")),
      &thisId, y.bg);
  thisId.fIdentifier = "P";
  d_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 3, "P")),
      &thisId, y.P);
  thisId.fIdentifier = "Q";
  d_emlrt_marshallIn(
      sp, emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 4, "Q")),
      &thisId, y.Q);
  thisId.fIdentifier = "p_ecef0";
  emlrt_marshallIn(
      sp,
      emlrtAlias(emlrtGetFieldR2017b((emlrtConstCTX)&sp, u, 0, 5, "p_ecef0")),
      &thisId, y.p_ecef0);
  emlrtDestroyArray(&u);
}

static void emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                             const emlrtMsgIdentifier *parentId, real_T y[3])
{
  e_emlrt_marshallIn(sp, emlrtAlias(u), parentId, y);
  emlrtDestroyArray(&u);
}

static const mxArray *emlrt_marshallOut(const struct4_T &u)
{
  static const int32_T i{3};
  static const int32_T i1{4};
  static const int32_T i2{3};
  static const int32_T i3{3};
  static const char_T *sv[6]{"p", "q", "bg", "P", "Q", "p_ecef0"};
  const mxArray *b_y;
  const mxArray *c_y;
  const mxArray *d_y;
  const mxArray *e_y;
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  y = nullptr;
  emlrtAssign(&y, emlrtCreateStructMatrix(1, 1, 6, (const char_T **)&sv[0]));
  b_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.p[0];
  pData[1] = u.p[1];
  pData[2] = u.p[2];
  emlrtAssign(&b_y, m);
  emlrtSetFieldR2017b(y, 0, "p", b_y, 0);
  c_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i1, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.q[0];
  pData[1] = u.q[1];
  pData[2] = u.q[2];
  pData[3] = u.q[3];
  emlrtAssign(&c_y, m);
  emlrtSetFieldR2017b(y, 0, "q", c_y, 1);
  d_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i2, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.bg[0];
  pData[1] = u.bg[1];
  pData[2] = u.bg[2];
  emlrtAssign(&d_y, m);
  emlrtSetFieldR2017b(y, 0, "bg", d_y, 2);
  emlrtSetFieldR2017b(y, 0, "P", emlrt_marshallOut(u.P), 3);
  emlrtSetFieldR2017b(y, 0, "Q", emlrt_marshallOut(u.Q), 4);
  e_y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i3, mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  pData[0] = u.p_ecef0[0];
  pData[1] = u.p_ecef0[1];
  pData[2] = u.p_ecef0[2];
  emlrtAssign(&e_y, m);
  emlrtSetFieldR2017b(y, 0, "p_ecef0", e_y, 5);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[100])
{
  static const int32_T iv[2]{10, 10};
  const mxArray *m;
  const mxArray *y;
  real_T *pData;
  int32_T i;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  pData = emlrtMxGetPr(m);
  i = 0;
  for (int32_T b_i{0}; b_i < 10; b_i++) {
    for (int32_T c_i{0}; c_i < 10; c_i++) {
      pData[i + c_i] = u[c_i + 10 * b_i];
    }
    i += 10;
  }
  emlrtAssign(&y, m);
  return y;
}

static void f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[9])
{
  static const int32_T dims[2]{3, 3};
  real_T(*r)[9];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[9])emlrtMxGetData(src);
  for (int32_T i{0}; i < 9; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

static void g_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[4])
{
  static const int32_T dims{4};
  real_T(*r)[4];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                          (const void *)&dims);
  r = (real_T(*)[4])emlrtMxGetData(src);
  ret[0] = (*r)[0];
  ret[1] = (*r)[1];
  ret[2] = (*r)[2];
  ret[3] = (*r)[3];
  emlrtDestroyArray(&src);
}

static void h_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                               const emlrtMsgIdentifier *msgId, real_T ret[100])
{
  static const int32_T dims[2]{10, 10};
  real_T(*r)[100];
  emlrtCheckBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                          (const void *)&dims[0]);
  r = (real_T(*)[100])emlrtMxGetData(src);
  for (int32_T i{0}; i < 100; i++) {
    ret[i] = (*r)[i];
  }
  emlrtDestroyArray(&src);
}

void EKF_func_api(const mxArray *const prhs[4], const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  struct0_T IMU;
  struct2_T GNSS;
  struct3_T Vehicle;
  struct4_T state;
  st.tls = emlrtRootTLSGlobal;
  // Marshall function inputs
  emlrt_marshallIn(st, emlrtAliasP(prhs[0]), "IMU", IMU);
  emlrt_marshallIn(st, emlrtAliasP(prhs[1]), "GNSS", GNSS);
  Vehicle = emlrt_marshallIn(st, emlrtAliasP(prhs[2]), "Vehicle");
  emlrt_marshallIn(st, emlrtAliasP(prhs[3]), "state", state);
  // Invoke the target function
  EKF_func(&IMU, &GNSS, &Vehicle, &state);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(state);
}

void EKF_func_atexit()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtEnterRtStackR2012b(&st);
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
  EKF_func_xil_terminate();
  EKF_func_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void EKF_func_initialize()
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  mexFunctionCreateRootTLS();
  st.tls = emlrtRootTLSGlobal;
  emlrtClearAllocCountR2012b(&st, false, 0U, nullptr);
  emlrtEnterRtStackR2012b(&st);
  emlrtFirstTimeR2012b(emlrtRootTLSGlobal);
}

void EKF_func_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

// End of code generation (_coder_EKF_func_api.cpp)
