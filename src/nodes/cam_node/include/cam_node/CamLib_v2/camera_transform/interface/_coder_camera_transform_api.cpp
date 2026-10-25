//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_camera_transform_api.cpp
//
// Code generation for function 'initializeCameraToCoGTransform'
//

// Include files
#include "_coder_camera_transform_api.h"
#include "_coder_camera_transform_mex.h"

// Variable Definitions
emlrtCTX emlrtRootTLSGlobal{nullptr};

emlrtContext emlrtContextGlobal{
    true,                                                 // bFirstTime
    false,                                                // bInitialized
    131643U,                                              // fVersionInfo
    nullptr,                                              // fErrorFunction
    "camera_transform",                                   // fFunctionName
    nullptr,                                              // fRTCallStack
    false,                                                // bDebugMode
    {2045744189U, 2170104910U, 2743257031U, 4284093946U}, // fSigWrd
    nullptr                                               // fSigMem
};

// Function Declarations
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16];

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16];

static const mxArray *b_emlrt_marshallOut(const real_T u[3]);

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[4];

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4];

static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3];

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3];

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3];

static const mxArray *emlrt_marshallOut(const real_T u[16]);

static real_T (*f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4];

// Function Definitions
static real_T (*b_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[16]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[16];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = b_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*b_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[16]
{
  real_T(*y)[16];
  y = e_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *b_emlrt_marshallOut(const real_T u[3])
{
  static const int32_T i{0};
  static const int32_T i1{3};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(1, (const void *)&i, mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &i1, 1);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp,
                                   const mxArray *b_nullptr,
                                   const char_T *identifier))[4]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[4];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = c_emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*c_emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                   const emlrtMsgIdentifier *parentId))[4]
{
  real_T(*y)[4];
  y = f_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static real_T (*d_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[3]
{
  static const int32_T dims{3};
  real_T(*ret)[3];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[3])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*e_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[16]
{
  static const int32_T dims[2]{4, 4};
  real_T(*ret)[16];
  int32_T iv[2];
  boolean_T bv[2]{false, false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 2U,
                            (const void *)&dims[0], &bv[0], &iv[0]);
  ret = (real_T(*)[16])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *b_nullptr,
                                 const char_T *identifier))[3]
{
  emlrtMsgIdentifier thisId;
  real_T(*y)[3];
  thisId.fIdentifier = const_cast<const char_T *>(identifier);
  thisId.fParent = nullptr;
  thisId.bParentIsCell = false;
  y = emlrt_marshallIn(sp, emlrtAlias(b_nullptr), &thisId);
  emlrtDestroyArray(&b_nullptr);
  return y;
}

static real_T (*emlrt_marshallIn(const emlrtStack &sp, const mxArray *u,
                                 const emlrtMsgIdentifier *parentId))[3]
{
  real_T(*y)[3];
  y = d_emlrt_marshallIn(sp, emlrtAlias(u), parentId);
  emlrtDestroyArray(&u);
  return y;
}

static const mxArray *emlrt_marshallOut(const real_T u[16])
{
  static const int32_T iv[2]{0, 0};
  static const int32_T iv1[2]{4, 4};
  const mxArray *m;
  const mxArray *y;
  y = nullptr;
  m = emlrtCreateNumericArray(2, (const void *)&iv[0], mxDOUBLE_CLASS, mxREAL);
  emlrtMxSetData((mxArray *)m, (void *)&u[0]);
  emlrtSetDimensions((mxArray *)m, &iv1[0], 2);
  emlrtAssign(&y, m);
  return y;
}

static real_T (*f_emlrt_marshallIn(const emlrtStack &sp, const mxArray *src,
                                   const emlrtMsgIdentifier *msgId))[4]
{
  static const int32_T dims{4};
  real_T(*ret)[4];
  int32_T i;
  boolean_T b{false};
  emlrtCheckVsBuiltInR2012b((emlrtConstCTX)&sp, msgId, src, "double", false, 1U,
                            (const void *)&dims, &b, &i);
  ret = (real_T(*)[4])emlrtMxGetData(src);
  emlrtDestroyArray(&src);
  return ret;
}

void camera_transform_atexit()
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
  camera_transform_xil_terminate();
  camera_transform_xil_shutdown();
  emlrtExitTimeCleanup(&emlrtContextGlobal);
}

void camera_transform_initialize()
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

void camera_transform_terminate()
{
  emlrtDestroyRootTLS(&emlrtRootTLSGlobal);
}

void initializeCameraToCoGTransform_api(const mxArray *const prhs[4],
                                        const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*T_body_camera)[16];
  real_T(*camera_dir_V)[3];
  real_T(*camera_pos_V)[3];
  real_T(*camera_up_V)[3];
  real_T(*cog_pos_V)[3];
  st.tls = emlrtRootTLSGlobal;
  T_body_camera = (real_T(*)[16])mxMalloc(sizeof(real_T[16]));
  // Marshall function inputs
  camera_pos_V = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "camera_pos_V");
  camera_dir_V = emlrt_marshallIn(st, emlrtAlias(prhs[1]), "camera_dir_V");
  camera_up_V = emlrt_marshallIn(st, emlrtAlias(prhs[2]), "camera_up_V");
  cog_pos_V = emlrt_marshallIn(st, emlrtAlias(prhs[3]), "cog_pos_V");
  // Invoke the target function
  initializeCameraToCoGTransform(*camera_pos_V, *camera_dir_V, *camera_up_V,
                                 *cog_pos_V, *T_body_camera);
  // Marshall function outputs
  *plhs = emlrt_marshallOut(*T_body_camera);
}

void transformBodyToENU_api(const mxArray *const prhs[3], const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  const mxArray *prhs_copy_idx_2;
  real_T(*q)[4];
  real_T(*p_body)[3];
  real_T(*p_cog_enu)[3];
  real_T(*p_enu)[3];
  st.tls = emlrtRootTLSGlobal;
  p_enu = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  prhs_copy_idx_2 = emlrtProtectR2012b(prhs[2], 2, false, -1);
  // Marshall function inputs
  p_body = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "p_body");
  p_cog_enu = emlrt_marshallIn(st, emlrtAlias(prhs[1]), "p_cog_enu");
  q = c_emlrt_marshallIn(st, emlrtAlias(prhs_copy_idx_2), "q");
  // Invoke the target function
  transformBodyToENU(*p_body, *p_cog_enu, *q, *p_enu);
  // Marshall function outputs
  *plhs = b_emlrt_marshallOut(*p_enu);
}

void transformCameraToCoG_api(const mxArray *const prhs[2],
                              const mxArray **plhs)
{
  emlrtStack st{
      nullptr, // site
      nullptr, // tls
      nullptr  // prev
  };
  real_T(*T_body_camera)[16];
  real_T(*p_body)[3];
  real_T(*p_camera)[3];
  st.tls = emlrtRootTLSGlobal;
  p_body = (real_T(*)[3])mxMalloc(sizeof(real_T[3]));
  // Marshall function inputs
  p_camera = emlrt_marshallIn(st, emlrtAlias(prhs[0]), "p_camera");
  T_body_camera = b_emlrt_marshallIn(st, emlrtAlias(prhs[1]), "T_body_camera");
  // Invoke the target function
  transformCameraToCoG(*p_camera, *T_body_camera, *p_body);
  // Marshall function outputs
  *plhs = b_emlrt_marshallOut(*p_body);
}

// End of code generation (_coder_camera_transform_api.cpp)
