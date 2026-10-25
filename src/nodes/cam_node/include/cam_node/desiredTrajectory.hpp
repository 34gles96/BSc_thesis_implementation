#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "readConfig.hpp"
#include "fitLanePoly.hpp"
#include <limits>

// Important: These XYZ coordinates are the calculated desired trajectory in the camera frame, not the measured stereo depth at those pixels.

struct DesiredTrajectory
{
    // Ordered desired trajectory pixels
    std::vector<cv::Point> uv_pixels;

    // Corresponding XYZ points in camera frame
    std::vector<cv::Vec3f> xyz_points;

    // Mapping: xyz_map(v,u) = (X,Y,Z)
    cv::Mat_<cv::Vec3f> xyz_map;
};

// Draw trajectory and lane line on the original image frame
DesiredTrajectory drawDesiredTrajectoryOnImage(

    cv::Mat &original_bgr_frame,
    double a2, double a1, double a0, // Polynomial coeffs: X(Z) = a2*Z^2 + a1*Z + a0
    double road_height_y,            // Camera height relative to road (in camera frame, Y is down)
    double &lane_width,              // e.g., 3.5 meters
    bool is_left_lane,               // true if left line, false if right line
    const CameraParams &cam,
    struct LanePolynomial lanepoly);
