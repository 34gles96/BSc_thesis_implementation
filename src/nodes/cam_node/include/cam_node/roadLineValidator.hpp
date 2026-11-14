#ifndef ROAD_LINE_VALIDATOR_HPP
#define ROAD_LINE_VALIDATOR_HPP

#include <opencv2/core.hpp>
#include <vector>

bool isValidRoadLine(
    const cv::Mat &XYZ_IMG,
    const std::vector<cv::Point> &lanePixels,
    double minDepth,
    double maxDepth,
    double binSize,
    double maxAllowedWidth,
    double minValidBinRatio = 0.6);

#endif