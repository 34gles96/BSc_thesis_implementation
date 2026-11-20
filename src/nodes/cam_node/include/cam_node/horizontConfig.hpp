#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <optional>

struct TrajectoryBounds
{
    double min_z;
    double predicted_horizon;
};

std::optional<TrajectoryBounds> calculateTrajectoryBoundsFromLane(
    const cv::Mat &XYZ_IMG,
    const std::vector<cv::Point> &selected_lane);