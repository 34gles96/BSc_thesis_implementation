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
  const char_T *data[9]{
      "789ced98bf6fd34014c75d7e0f403b000216241658da3421b4295b9bd86968eaa64dd240"
      "2a482fce35bdc4be737c767f648299012624c482c48c045b07061624"
      "241840624042c0c07f818444d3e49adac24a91a32b71fba4eafafa4dfa7df74efee4f284"
      "bec4749f2008a78566e883cdf5542b1f68ad87047b38f5bed67ac291",
      "b3382a1cb1bd8fe90f5bab42b009d7cc66828106b7df59221ac2009b99751d0a06a4445d"
      "81a52d6509a930833498de99c88d4c937648db49436afc1e5d864a35"
      "6d6982b14cdb15aa3b93ed7ebc74d9ef915df64376e9c780435f10ef04b2141a34a0035d"
      "47d45c01381023558aaa015a07d51251cba40ecc80064c15140b48d3",
      "55a8416c0213114c03e294d4f8292c595819d2daf5af79acff6c87fa998e30321150511d"
      "1644395b20062a232cb4eb58f458c731d73a9a4a89584515b6fdde78"
      "f45b70f5b3ebdd38b7bff6ae718842e7fef5ef723fceb5fdfae6133bb8f1bec2d32f177c"
      "7099a71f8bbdf2f3fa1c9e73f11b70e8c9d5c49a1acbcb647972343a",
      "97cc865345459b6cd791eae0d3a90ec125e7f5fff73b8fcf77a89fe98a0181090b89e96c"
      "a8484aeb0583342beb551edf75f5b3ebdd3837b7deed3cc745977abb"
      "c58f1f1ff9f2f8cbf3c3d778fab1f03b8fe7954cb034a75bf5702e1db91dcfcf2c8d9965"
      "f180c77ee1f1990ef5335d87c61231b40654caeb06d9024be3efbdca",
      "e3bcab9f5defc6b9fdad77adeb31bffbf127be3c7ef5337d83a71f0bbff3381c9c8bc9c3"
      "2b21098f9893523d1203723577c0e3ff86c7bac7fafb1db9b37ea6eb"
      "062c21a5514b819a50e7c6616a1a9662b6f7bbe1d12febea67d7bbc2617bcfb610cc8b17"
      "6f9f7de6cadf4b93bf4ef2f463e177fece8fdf5a8e907004668ada54",
      "249a9b95921979c23ffcf5cbf37ccfe33e2e74d807d315a02a96baf9753bbe79b34b1944"
      "8194ca045136ffefd5fb71c1d5cfae77655ee1d6c3cd13e5c593079c"
      "f9fce3e9ebe33cfd58f89dcf332024295208c562297574ba765b898eaba1a87ff8bcdfef"
      "c76efd1a70acec3b7732391e12a3a2b4571cf6fa793aefea67d7bb39",
      "a7603de3392fae7ce33b9f18fc3a7a85a71f0bbff337b95a99b5f0683c53be3e2d47c45a"
      "79241e560ff8eb1bfeee763ea110c38007f3897fbb07db7bc6753e71"
      "fff707aefc7df7e8c9559e7e2cfccedf557893de9cad5dc7563928a6e667f2c3b160d547"
      "f361bf3ccff73ceee362877d30bd6401358e299d0014aa08c304c664",
      "65abc6a6deabf38945472e385ec7f46e9ca37b0f87345e3c79cc99cfe28bef433cfd58f8"
      "9dcf248b526395742d970579299d48466be111c9077cfe03e560355a",
      ""};
  nameCaptureInfo = nullptr;
  emlrtNameCaptureMxArrayR2016a(&data[0], 11656U, &nameCaptureInfo);
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
                emlrtMxCreateString("qXSveAH4UaKHTXvKoS5GJD"));
  emlrtSetField(xResult, 0, "EntryPoints", xEntryPoints);
  return xResult;
}

// End of code generation (_coder_EKF_func_info.cpp)
