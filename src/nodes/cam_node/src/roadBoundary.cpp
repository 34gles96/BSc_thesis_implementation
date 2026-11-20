
#include "road_boundary_fitting.hpp"

#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <cmath>
#include <iostream>
#include <numeric>
#include "readConfig.hpp"
#include "detectObstacles.hpp"

constexpr double MAX_RMSE_M = 0.15; // Initial threshold; tune experimentally

std::vector<cv::Point2d> extractRepresentativePoints(
    const cv::Mat &whiteLineXYZ,
    const std::vector<cv::Point> &selected_lane,
    double minDepth,
    double maxDepth,
    double binSize)
{
    std::map<int, std::vector<double>> depthBins;

    if (binSize <= 0.0 || maxDepth <= minDepth)
        return {};

    for (const cv::Point &pixel : selected_lane)
    {
        // Check pixel coordinates before accessing the matrix
        if (pixel.x < 0 || pixel.x >= whiteLineXYZ.cols ||
            pixel.y < 0 || pixel.y >= whiteLineXYZ.rows)
            continue;

        cv::Vec3f pt3D =
            whiteLineXYZ.at<cv::Vec3f>(pixel.y, pixel.x);

        const float X = pt3D[0];
        const float Z = pt3D[2];

        if (!std::isfinite(X) || !std::isfinite(Z))
            continue;

        if (Z < minDepth || Z >= maxDepth)
            continue;

        // Calculate the depth-bin index
        const int binIndex =
            static_cast<int>(std::floor((Z - minDepth) / binSize));

        depthBins[binIndex].push_back(X);
    }

    std::vector<cv::Point2d> XZ;

    // Calculate one representative point for each depth bin
    for (const auto &[binIndex, xValues] : depthBins)
    {
        if (xValues.empty())
            continue;

        const double averageX =
            std::accumulate(xValues.begin(), xValues.end(), 0.0) / xValues.size();

        const double Zcenter =
            minDepth + (binIndex + 0.5) * binSize;

        // Point.x = Z, Point.y = X
        XZ.emplace_back(Zcenter, averageX);
    }

    return XZ;
}

std::vector<double> secondOrderPolyFitting(
    const std::vector<cv::Point2d> &XZ)
{
    if (XZ.size() < 5) // 2nd order poly fttin needs at least 3 points but physicly unreasonable, safety guard at least 5
        return {};

    // ============================================================
    // 1. Determine the depth interval of the representative points
    // ============================================================

    double minZ = XZ.front().x;
    double maxZ = XZ.front().x;

    for (const cv::Point2d &point : XZ)
    {
        const double Z = point.x;
        const double X = point.y;

        if (!std::isfinite(Z) || !std::isfinite(X))
            return {};

        minZ = std::min(minZ, Z);
        maxZ = std::max(maxZ, Z);
    }

    const double span = maxZ - minZ;
    if (span < 0.75)
    { // small depth span -> too little curvature signal
        std::cerr << "small depth span -> too little curvature signal"
                  << XZ.size() << '\n';
        return {};
    }
    // ============================================================
    // 2. Calculate normalization parameters
    // ============================================================

    const double Zc = 0.5 * (maxZ + minZ);
    const double L = 0.5 * (maxZ - minZ);

    if (!std::isfinite(Zc) ||
        !std::isfinite(L) ||
        L < 1e-9)
    {
        return {};
    }

    // ============================================================
    // 3. Construct normalized least-squares system
    //
    // X(s) = b2*s^2 + b1*s + b0
    //
    // s = (Z - Zc) / L
    // ============================================================

    cv::Mat A(static_cast<int>(XZ.size()), 3, CV_64F);
    cv::Mat B(static_cast<int>(XZ.size()), 1, CV_64F);

    int i = 0;

    for (const cv::Point2d &point : XZ)
    {
        const double Z = point.x;
        const double X = point.y;

        const double s = (Z - Zc) / L;

        A.at<double>(i, 0) = s * s;
        A.at<double>(i, 1) = s;
        A.at<double>(i, 2) = 1.0;

        B.at<double>(i, 0) = X;

        ++i;
    }

    // ============================================================
    // 4. Solve the normalized least-squares problem
    // ============================================================

    cv::Mat coeffs;

    const bool success =
        cv::solve(A, B, coeffs, cv::DECOMP_SVD);

    if (!success || coeffs.rows != 3)
        return {};

    const double b2 = coeffs.at<double>(0, 0);
    const double b1 = coeffs.at<double>(1, 0);
    const double b0 = coeffs.at<double>(2, 0);

    if (!std::isfinite(b2) ||
        !std::isfinite(b1) ||
        !std::isfinite(b0))
    {
        return {};
    }

    // ============================================================
    // 5. Convert normalized coefficients back to original Z
    //
    // X(Z) = a2*Z^2 + a1*Z + a0
    // ============================================================

    const double a2 = b2 / (L * L);

    const double a1 =
        b1 / L - (2.0 * b2 * Zc) / (L * L);

    const double a0 =
        b0 - (b1 * Zc) / L + (b2 * Zc * Zc) / (L * L);

    if (!std::isfinite(a2) ||
        !std::isfinite(a1) ||
        !std::isfinite(a0))
    {
        return {};
    }

    // ============================================================
    // 4.1 VALIDATE POLYNOMIAL FIT USING RMSE
    // ============================================================

    double squared_error_sum = 0.0;

    for (const cv::Point2d &point : XZ)
    {
        const double Z = point.x;
        const double X = point.y;

        const double s = (Z - Zc) / L;

        // Predicted X using normalized polynomial
        const double X_pred =
            b2 * s * s + b1 * s + b0;

        const double error = X - X_pred;

        squared_error_sum += error * error;
    }

    const double RMSE =
        std::sqrt(squared_error_sum / XZ.size());

    std::cout << "[Polyfit] RMSE: " << RMSE << " m" << std::endl;

    if (!std::isfinite(RMSE) || RMSE > MAX_RMSE_M)
    {
        std::cerr << "[Polyfit] Rejected: fitting error too large!"
                  << std::endl;

        return {};
    }

    // Return coefficients in the original polynomial form
    return {a2, a1, a0};
}
