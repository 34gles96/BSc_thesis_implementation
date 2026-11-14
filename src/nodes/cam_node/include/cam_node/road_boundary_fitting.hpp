
#ifndef ROAD_BOUNDARY_FITTING_HPP
#define ROAD_BOUNDARY_FITTING_HPP

#include <opencv2/core.hpp>
#include <vector>

// Extracts one representative (Z, X) point per depth bin
std::vector<cv::Point2d> extractRepresentativePoints(
    const cv::Mat &whiteLineXYZ,
    const std::vector<cv::Point> &selected_lane,
    double minDepth,
    double maxDepth,
    double binSize);

// Fits a second-order polynomial:
// X(Z) = a2 * Z^2 + a1 * Z + a0
// Returns coefficients in the order {a2, a1, a0}
std::vector<double> secondOrderPolyFitting(
    const std::vector<cv::Point2d> &XZ);

#endif // ROAD_BOUNDARY_FITTING_HPP
