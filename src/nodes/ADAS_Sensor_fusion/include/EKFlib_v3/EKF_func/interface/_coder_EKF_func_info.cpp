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
  const char_T *data[10]{
      "789ced5acf6fdb541c77d9604302d6039b18428090105cd63459d666bbb5899d664b9db4"
      "491a9669731dfb357d8dfd9ee367774d4ee386b4c33821212e489c91"
      "e036210e5c9090e030240e480876d87f310989e6c76b628b8737d97a6d4cbe52f4f2cdc7"
      "cee7f3be2ffee8eb170b7385f53941105e1386f1eee5e1f8ea289f1f",
      "8d2f08def0e373a3f1655f4ee345e1b4e73c8a3f188d1a460e38708609524d7074a68e4d"
      "8854e454bb16106c40b0b10ff401b2030d508526a84c26723f33a509"
      "e828e943fdf7d95da0b52bae29d8bb64acd0984c8eeaf10d63bea79fb11e32a31ef33efc"
      "96783b5123c026094bb52c489c7d152572b84d603b417a6a5bc7460b",
      "f7542761aa8ea13615685a063001725407624412e20da9ff52765ca42d9863fd0721f59f"
      "0fd04f7188a0035503f68022ca3505dbb0059130d6b11d52c74b4c1d"
      "4344c76ed30063be1f42f2dd62f279f128d6ed5f6bd75f4421b87ee79e713efe717cfcd9"
      "c178e9e1cf7b3cf9eac9fbeff1e4a3715c7c61afc30b0cbe791f5ebc",
      "5b3830720d19efae2d67378bb574b9a9996b631de5009e201d0223e7f5fdd3eec75648fd"
      "ac7acdfb460bd83bd83695627125256645e9b87cf86148be2d269f17"
      "8f62bdfc359b5cb76d86cea8fc62ef4fbefe7be98fe50f78f2d188bfffee6db868395f6d"
      "5d59973362a7b5944f1bd999ff9e14fffd24a4fef703f4537cdcd395",
      "6c4865493636f3889055950003a2fe3dce71f9f2a3907c6d269f178fb63ffe8f5a72ee97"
      "dffeee37ae7efd405e7c8b271f8db8fb7577552fc16e7513a51cb454"
      "efd41cb1b1d8cccdfcfaa4f875d8dfc11b01fa29aed94075805258afa59a58ef2a361e2a"
      "9bd6fd8b3b4c3e2f1ec5bab16ac7b37f7efc886ffffcfb57a72ef3e4",
      "a311773fded2aa497dd3727be97a257333df28ed5c755ae2cc8fe3e2c7af07e8a738bd17"
      "3f349556d7c60363e97f3ead7edc60f279f128f731266b376a8ff9ed"
      "2ffccad78fbf7d52b9c6938f46dcfd389ddcccc98bfb29092d396b522f9353e5767de6c7"
      "27c68fc3ee279ff3e57efd14b76ca043adaf45210eb0b8f930716c57",
      "73a2db4fae31f9bc78243eecadd9c08279f9c58f5ff2dd9f7867ede92b3cf968c4dd7fb7"
      "563edacde07406549be68d4cb6be2115abf26a7cfc372ed7f3bd90f3"
      "b818300f8a6baaa1b9c6e1ed76feb0b32bdb580384c81812fabcccb4f6c70a93cf8b47b2"
      "5fc1aae1e18af2f293fb9cfdf9f117df9fe1c94723eefe5c52539226",
      "a5602e573696d73b37b5ec8a919afddff7bfeb8f356cdb60d61f3f9f0f7b6bc6b53ffef8"
      "ef5fb8faef4f9f7efe214f3e1a71f7dfbbe03ab9bed1b982dc56522c"
      "6f951a8bb9643b46fb1371b99eef859cc79b01f3a0b8eeaac6e463010584f0fe40e3109f"
      "d6fe78db970bbee3281ec53ab26bb860f2f293cf38fbb3f8f55f0b3c",
      "f968c4dd9f710d96afee553af59ada902a8562b6935e9262e0cfff00ecaea410",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 13112U, &nameCaptureInfo);
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
                emlrtMxCreateDoubleScalar(740273.59902777779));
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
                emlrtMxCreateString("sBg8govjlvj1gnTzet92c"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_EKF_func_info.cpp)
