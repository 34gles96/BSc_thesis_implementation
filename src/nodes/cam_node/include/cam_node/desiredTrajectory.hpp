#pragma once

#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>
#include "readConfig.hpp"
#include "fitLanePoly.hpp"

// Draw trajectory and lane line on the original image frame
void drawDesiredTrajectoryOnImage(

    cv::Mat &original_bgr_frame,
    double a2, double a1, double a0, // Polynomial coeffs: X(Z) = a2*Z^2 + a1*Z + a0
    double road_height_y,            // Camera height relative to road (in camera frame, Y is down)
    double &lane_width,              // e.g., 3.5 meters
    bool is_left_lane,               // true if left line, false if right line
    const CameraParams &cam,
    struct LanePolynomial lanepoly);
