//
// Academic License - for use in teaching, academic research, and meeting
// course requirements at degree granting institutions only.  Not for
// government, commercial, or other organizational use.
//
// _coder_EKF_func_info.cpp
//
// Code generation for function 'EKF_func'
//

// Include files
#include "_coder_EKF_func_info.h"
#include "emlrt.h"
#include "tmwtypes.h"

// Function Declarations
static const mxArray *emlrtMexFcnResolvedFunctionsInfo();

// Function Definitions
static const mxArray *emlrtMexFcnResolvedFunctionsInfo()
{
  const mxArray *nameCaptureInfo;
  const char_T *data[8]{
      "789ced98cd6f124118c6b75aad172d0735c6a3977a692995b6d45b855d4a4bb7b4408934"
      "4a97dd0106766796fda02d273d9ba827134f1e3c9ba8a75e4c3c9ae8"
      "a1261e4c8c7af0ef30310b4c6137dd50b3646a57dea419de3ec0f3cc3be197c9322389d5"
      "1186612e31ed3a9868af173b7da0b39e61ece5d4473aeb98a327758e",
      "19b57d8ee84f3aab889101768d768304051c7e52c20a440232327b2a6034a063b901a496"
      "528232c84005a47b1bdeea14ae473a6c2cc97a1dad00b196361546ab"
      "e8dd84726f73388fd72efb1d3de63c789779041cfa167b2f98d581a607554155a16e3404"
      "148ce19a0e6b41bd29d4242c97715330828a60c842b1001555060a40",
      "8660408cf420bbc2597f859289c429a59b7fd763fe2b7df2131d22684041864d5060f96c"
      "016bb00c11d3cdb1ed31c779d71c6d45c26651065dbff71efdb65cfd"
      "ecfa20ceedc8d95987c8f49fdff831f7e35cbbefbfd05a27f73f5669fae5428f6ed0f423"
      "75527e5e7f87575dfc020e3db993d89563791e5796e6a31bc96c3855",
      "1495a56e8e541f9f7e3918979ed6f7ffef3cbed6273fd1450d0806282456b333452ced15"
      "34dc4e765a797cdfd5cfae0fe2dcdc66d77b8edb2e7907c58f9f0774"
      "79fcf5e5d95b34fd48f99dc79b6226246da866339c4b47eec6f36ba505a3cc0e79ec171e"
      "5fee939fe82ad04a58532ca894f734dc028bf5ffd3cae3bcab9f5d1f",
      "c4b91d35bbcef598defdf8335d1ebff995be4dd38f94df791c0e6dc4f8e9c60c87e68c25"
      "ae1989097c2d37e4f13fc363d563fe7147efcc4f7455031214ad2c05"
      "dd002a350eeb86668a4677bffb1efdb2ae7e767d201cb6cfac85605abc78f7852e7f1fbf"
      "7d3146d38f94dff98b339508e611ae85b995f5c56a535896a6e7e243",
      "fefa85bf6ef30a385672a74b261767d828cb9dd43dd82b7f375dfdecfa20efc16466349f"
      "4754bfd3e5efe4b7f9099a7ea4fccedfe44e75dd44f3f14c7976958f"
      "b0f5f25c3c2c4787fcf50b7f8f7bff15b1a681e1fdf7ef9e0bdb6746f5fefbf0f727aafc"
      "fdf0f4f94d9a7ea4fccedf1db0ac2fafd76791590eb1a9cdb5fc742c",
      "54f3d1f307bffc9e1f78dcc7f53efb20ba640a721ce9fa1d410732442081106eb432b6f5"
      "d3fa7c78dbd1338ef7117d10e7e83ec32985164f9e51e633fbeac714"
      "4d3f527ee733cec2d442355dcf65853c974e24a3f5f01ce7033eff019902cde0",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 10216U, &nameCaptureInfo);
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
  emlrtSetField(xEntryPoints, 0, "Name", emlrtMxCreateString("EKF_func"));
  emlrtSetField(xEntryPoints, 0, "NumberOfInputs",
                emlrtMxCreateDoubleScalar(4.0));
  emlrtSetField(xEntryPoints, 0, "NumberOfOutputs",
                emlrtMxCreateDoubleScalar(1.0));
  emlrtSetField(xEntryPoints, 0, "ConstantInputs", xInputs);
  emlrtSetField(xEntryPoints, 0, "FullPath",
                emlrtMxCreateString("/Users/pappistvan/Doksik/szakdolgozat/"
                                    "matlab_implementations/EKF/EKF_func.m"));
  emlrtSetField(xEntryPoints, 0, "TimeStamp",
                emlrtMxCreateDoubleScalar(740273.45584490744));
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
                emlrtMxCreateString("kbBkkdkrDIzTvcyhg4Klc"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_EKF_func_info.cpp)
