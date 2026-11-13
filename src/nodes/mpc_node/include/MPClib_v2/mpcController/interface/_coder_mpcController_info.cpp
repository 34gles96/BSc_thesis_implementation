//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_mpcController_info.cpp
//
// Code generation for function 'mpcController'
//

// Include files
#include "_coder_mpcController_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[5]{
      "789cd555494ec330147510d382a10b8458738046a02260092d0884c2a032882254dce417"
      "4c3c84d8542d97e0268823b0618fd881b8019780269d1261b552aba0"
      "fecdcfd3b3fdfee07c2363d73210423328b485a9d04f3770aae14750d4e2bc115b674497"
      "a331341ad9d7e41f1bde165c415585806306ad9d8e608463ae8e6b1e",
      "201fa4a0157002a64c281c1306f94eb05f476cbb836a813a55ffcede80ede6ef19f26f64"
      "3b42da095af578d3e43bda633d6e35f548c5f88bad4bf344822f4d0f"
      "7b1e91aa82b99913ae24ae291fb0eb087a2d1eb032195614978a84791418708515115c9a"
      "d661d6649e9dfd2da32f28053fcd8c082e622ef8b6cd97a2f95df599",
      "df8436bf900962906a607ae35abd9071c47d89423bbfe73ef52cad5e941f58ffa44a33d4"
      "bd3fb33dc61ff7edf59381ff565fc1154c4aeff5fd6d2d49bda6fd97"
      "5e55735eaff76f4ea3978af18553b20147705ea86c2e974bf942d95d669d711c76d1e916"
      "07d2e0a4ce1ff6ff38a97b502276cda6600907688e48db07153ca5ff",
      "357f5ffad42b68f5a2fc20faf657ed8226a2e4e6c767c2f3f8c9ff584c52af69c33a8fe7"
      "357aa9187fbdb76afb2493b1722b07ac4ad99de2eb673bc33f8f7f00"
      "7e70e9a4",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 3120U, &nameCaptureInfo);
  return nameCaptureInfo;
}

mxArray *emlrtMexFcnProperties()
{
  mxArray *xEntryPoints;
  mxArray *xInputs;
  mxArray *xResult;
  const char_T *propFieldName[9]{"Version",
                                 "ResolvedFunctions",
                                 "Checksum",
                                 "EntryPoints",
                                 "CoverageInfo",
                                 "IsPolymorphic",
                                 "PropertyList",
                                 "UUID",
                                 "ClassEntryPointIsHandle"};
  const char_T *epFieldName[8]{
      "Name",     "NumberOfInputs", "NumberOfOutputs", "ConstantInputs",
      "FullPath", "TimeStamp",      "Constructor",     "Visible"};
  xEntryPoints =
      emlrtCreateStructMatrix(1, 1, 8, (const char_T **)&epFieldName[0]);
  xInputs = emlrtCreateLogicalMatrix(1, 4);
  emlrtSetField(xEntryPoints, 0, "Name", emlrtMxCreateString("mpcController"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(2.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(
      xEntryPoints, 0, "FullPath",
      emlrtMxCreateString("/Users/pappistvan/Doksik/szakdolgozat/"
                          "matlab_implementations/MPC/mpcController.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740289.9456597222));
  emlrtSetField(xEntryPoints, 0, "Constructor",
                emlrtMxCreateLogicalScalar(false));
  emlrtSetField(xEntryPoints, 0, "Visible", emlrtMxCreateLogicalScalar(true));
  xResult =
      emlrtCreateStructMatrix(1, 1, 9, (const char_T **)&propFieldName[0]);
  emlrtSetField(xResult, 0, "Version",
                emlrtMxCreateString("23.2.0.2485118 (R2023b) Update 6"));
  emlrtSetField(xResult, 0, "ResolvedFunctions",
                (mxArray *)emlrtMexFcnResolvedFunctionsInfo());
  emlrtSetField(xResult, 0, "Checksum",
                emlrtMxCreateString("6pxgukZemhCBLh1wK1tG0G"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_mpcController_info.cpp)
