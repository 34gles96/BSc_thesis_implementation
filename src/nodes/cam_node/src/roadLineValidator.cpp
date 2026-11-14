#include "roadLineValidator.hpp"

#include <map>
#include <cmath>
#include <algorithm>

bool isValidRoadLine(
    const cv::Mat &XYZ_IMG,
    const std::vector<cv::Point> &lanePixels,
    double minDepth,
    double maxDepth,
    double binSize,
    double maxAllowedWidth,
    double minValidBinRatio)
{
    if (binSize <= 0.0 || maxDepth <= minDepth ||
        lanePixels.empty())
        return false;

    std::map<int, std::vector<double>> depthBins;

    // Group lateral X coordinates according to depth
    for (const cv::Point &pixel : lanePixels)
    {
        if (pixel.x < 0 || pixel.x >= XYZ_IMG.cols ||
            pixel.y < 0 || pixel.y >= XYZ_IMG.rows)
            continue;

        cv::Vec3f pt3D =
            XYZ_IMG.at<cv::Vec3f>(pixel.y, pixel.x);

        const double X = pt3D[0];
        const double Z = pt3D[2];

        if (!std::isfinite(X) || !std::isfinite(Z))
            continue;

        if (Z < minDepth || Z >= maxDepth)
            continue;

        const int binIndex =
            static_cast<int>(std::floor((Z - minDepth) / binSize));

        depthBins[binIndex].push_back(X);
    }

    int validBins = 0;
    int totalBins = 0;

    // Validate the lateral width of each populated bin
    for (const auto &[binIndex, xValues] : depthBins)
    {
        if (xValues.size() < 2)
            continue;

        ++totalBins;

        const auto [minIt, maxIt] =
            std::minmax_element(xValues.begin(), xValues.end());

        const double lateralWidth = *maxIt - *minIt;

        if (lateralWidth <= maxAllowedWidth)
            ++validBins;
    }

    if (totalBins == 0)
        return false;

    const double validRatio =
        static_cast<double>(validBins) / totalBins;

    return validRatio >= minValidBinRatio;
}