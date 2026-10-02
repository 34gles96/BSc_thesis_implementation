#pragma once

#include <yaml-cpp/yaml.h>

struct CameraParams
{
    double fx; // focal leght [pixels]
    double fy;
    double cx;
    double cy;
    double baselink;
    int width;
    int height;
};

CameraParams loadCameraParams(const std::string &filename);