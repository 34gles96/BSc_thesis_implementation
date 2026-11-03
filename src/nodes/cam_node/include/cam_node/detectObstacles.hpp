
#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include "fitLanePoly.hpp"

struct RoadPlane
{
    cv::Vec3d normal;
    double d;
};

std::vector<cv::Vec3d> matToVector(
    const cv::Mat_<cv::Vec3d> &pointCloud,
    const cv::Rect &roi,
    double minDepth,
    double maxDepth);
bool detectRoad(
    RoadPlane &rp,
    const cv::Mat_<cv::Vec3d> &XYZ_cam);

void detectObstacles(
    const cv::Mat_<cv::Vec3d> &XYZ_cam,
    const RoadPlane &roadplane,
    double obstacleThreshold,
    cv::Mat1d &heightMap,
    cv::Mat1b &obstacleMask,
    const cv::Vec3d &upDirection = cv::Vec3d(0.0, -1.0, 0.0));

void drawDetectedObstacles(
    const cv::Mat1b &obstacleMask,
    cv::Mat &leftImage);

void filterObstaclesToLane(
    const cv::Mat_<cv::Vec3d> &XYZ_cam,
    cv::Mat1b &obstacleMask,
    const LanePolynomial &roadPoly,
    double laneWidth);