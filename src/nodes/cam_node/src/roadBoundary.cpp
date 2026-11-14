
#include "road_boundary_fitting.hpp"

#include <opencv2/core.hpp>
#include <map>
#include <vector>
#include <cmath>
#include <numeric>

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
    if (XZ.size() < 3)
        return {};

    cv::Mat A(static_cast<int>(XZ.size()), 3, CV_64F);
    cv::Mat B(static_cast<int>(XZ.size()), 1, CV_64F);

    int i = 0;

    for (const cv::Point2d &point : XZ)
    {
        const double Z = point.x;
        const double X = point.y;

        A.at<double>(i, 0) = Z * Z;
        A.at<double>(i, 1) = Z;
        A.at<double>(i, 2) = 1.0;

        B.at<double>(i, 0) = X;

        i++;
    }

    cv::Mat coeffs;

    bool success = cv::solve(A, B, coeffs, cv::DECOMP_SVD);

    if (!success)
        return {};

    return {
        coeffs.at<double>(0, 0), // a2
        coeffs.at<double>(1, 0), // a1
        coeffs.at<double>(2, 0)  // a0
    };
}