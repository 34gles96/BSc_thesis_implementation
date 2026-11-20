#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <iostream>

// Forward declarations
struct CameraParams;
struct RoadPlane;

typedef struct LanePolynomial
{
    double a2 = 0.0;
    double a1 = 0.0;
    double a0 = 0.0;
    bool isValid = false;
    bool is_left_lane = true;
    bool is_road_width_Valid = false;
} LanePolynomial;

// first arg correspond to white images, 2nd is cv::Mat<cv::Vec3f>(u,v) = (X,Y,Z) mapping, 3rd arg is modifiyng road_witdh
std::vector<LanePolynomial> fitLanePolynomialOpenCV(
    const cv::Mat &uvIMG,
    const cv::Mat &XYZ_IMG, double &road_width,
    const CameraParams &cam,
    const RoadPlane &road_plane);
