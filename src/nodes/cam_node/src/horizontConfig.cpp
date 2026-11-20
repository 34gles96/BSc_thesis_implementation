#include "horizontConfig.hpp"

#include <limits>
#include <algorithm>
#include <cmath>
#include <iostream>

std::optional<TrajectoryBounds> calculateTrajectoryBoundsFromLane(
    const cv::Mat &XYZ_IMG,
    const std::vector<cv::Point> &selected_lane)
{
    // ======== Design the min trajectory due to Z ========

    std::vector<double> valid_depths;
    valid_depths.reserve(selected_lane.size());

    constexpr double MAX_DEPTH_CAP = 20.0;
    constexpr double MIN_DEPTH_CAP = 3.0;

    double farthest_z =
        -std::numeric_limits<double>::infinity();

    for (const auto &pt : selected_lane)
    {
        if (pt.x < 0 || pt.x >= XYZ_IMG.cols ||
            pt.y < 0 || pt.y >= XYZ_IMG.rows)
            continue;

        const cv::Vec3f xyz =
            XYZ_IMG.at<cv::Vec3f>(pt.y, pt.x);

        const float z = xyz[2];

        if (std::isfinite(xyz[0]) &&
            std::isfinite(xyz[1]) &&
            std::isfinite(z) &&
            z >= MIN_DEPTH_CAP &&
            z <= MAX_DEPTH_CAP)
        {
            valid_depths.push_back(z);

            farthest_z =
                std::max(farthest_z, static_cast<double>(z));
        }
    }

    if (valid_depths.size() < 3)
    {
        std::cerr << "Not enough valid lane depths: "
                  << valid_depths.size() << '\n';

        return std::nullopt;
    }

    // Use the approximately 1st percentile
    const std::size_t index_min =
        (valid_depths.size() - 1) / 99;

    std::nth_element(
        valid_depths.begin(),
        valid_depths.begin() + index_min,
        valid_depths.end());

    const double min_z =
        static_cast<double>(valid_depths[index_min]);

    // Calculate predicted horizon
    double tunning_farthest_z = 0.87;

    const double predicted_horizon =
        farthest_z * tunning_farthest_z;

    return TrajectoryBounds{
        min_z,
        predicted_horizon};
}