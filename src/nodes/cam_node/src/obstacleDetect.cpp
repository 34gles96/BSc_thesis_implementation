#include <algorithm>
#include <string>
#include <opencv2/core.hpp>
#include <vector>
#include <random>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "detectObstacles.hpp"
#include "fitLanePoly.hpp"
#include <iterator>
#include "config.hpp"

bool detectRoad(
    RoadPlane &rp,
    const cv::Mat_<cv::Vec3d> &XYZ_cam)
{
    constexpr double minRoadDepth = 8.0;
    constexpr double maxRoadDepth = 15.0;

    constexpr int iterations = 100;
    constexpr double threshold_road = 0.15;

    constexpr std::size_t MAX_RANSAC_POINTS = 12000;
    constexpr int MIN_FULL_INLIERS = 100;

    // ============================================================
    // Define vertical ROI for road-plane detection
    // ============================================================

    const int roi_top = static_cast<int>((
                                             XYZ_cam.rows * ::config::img_in_stream_vertical_cut_top) *
                                         1.1);

    const int roi_bottom = static_cast<int>((
                                                XYZ_cam.rows * ::config::img_in_stream_vertical_cut_bottom) *
                                            0.8);

    const cv::Rect roadROI(
        0,
        roi_top,
        XYZ_cam.cols,
        roi_bottom - roi_top);

    // -------------------------------------------------
    // 1. Extract all valid 3D points
    // -------------------------------------------------

    const std::vector<cv::Vec3d> pointCloud =
        matToVector(
            XYZ_cam,
            roadROI,
            minRoadDepth,
            maxRoadDepth);

    if (pointCloud.size() < 3)
        return false;

    // -------------------------------------------------
    // 2. Create a representative RANSAC subset
    // -------------------------------------------------

    std::mt19937 rng(42);

    const std::size_t sampleCount =
        std::min(MAX_RANSAC_POINTS, pointCloud.size());

    std::vector<cv::Vec3d> ransacPoints;
    ransacPoints.reserve(sampleCount);

    std::sample(
        pointCloud.begin(),
        pointCloud.end(),
        std::back_inserter(ransacPoints),
        sampleCount,
        rng);

    // -------------------------------------------------
    // 3. Configure RANSAC
    // -------------------------------------------------

    std::uniform_int_distribution<std::size_t> pointSelector(
        0,
        ransacPoints.size() - 1);

    const cv::Vec3d up_cam(0.0, -1.0, 0.0);

    int bestInlierCount = 0;

    cv::Vec3d bestNormal;
    double bestD = 0.0;

    bool modelFound = false;

    // -------------------------------------------------
    // 4. RANSAC candidate generation and scoring
    // -------------------------------------------------

    for (int i = 0; i < iterations; ++i)
    {
        const std::size_t a = pointSelector(rng);
        const std::size_t b = pointSelector(rng);
        const std::size_t c = pointSelector(rng);

        if (a == b || a == c || b == c)
            continue;

        const cv::Vec3d &p1 = ransacPoints[a];
        const cv::Vec3d &p2 = ransacPoints[b];
        const cv::Vec3d &p3 = ransacPoints[c];

        const cv::Vec3d v1 = p1 - p2;
        const cv::Vec3d v2 = p1 - p3;

        cv::Vec3d normal = v1.cross(v2);

        const double normalMagnitude = cv::norm(normal);

        if (normalMagnitude < 1e-9)
            continue;

        normal /= normalMagnitude;

        // Orient normal towards the upward direction.
        if (normal.dot(up_cam) < 0.0)
            normal = -normal;

        // Reject approximately vertical surfaces.
        if (normal.dot(up_cam) < 0.80)
            continue;

        const double d = -normal.dot(p1);

        // Evaluate candidate using only the RANSAC subset.
        int inlierCount = 0;

        for (const auto &point : ransacPoints)
        {
            const double distance =
                std::abs(normal.dot(point) + d);

            if (distance < threshold_road)
                ++inlierCount;
        }

        if (inlierCount > bestInlierCount)
        {
            bestInlierCount = inlierCount;

            bestNormal = normal;
            bestD = d;

            modelFound = true;
        }
    }

    if (!modelFound)
        return false;

    // -------------------------------------------------
    // 5. Validate the selected plane against ALL points
    // -------------------------------------------------

    int fullInlierCount = 0;

    for (const auto &point : pointCloud)
    {
        const double distance =
            std::abs(bestNormal.dot(point) + bestD);

        if (distance < threshold_road)
            ++fullInlierCount;
    }

    // -------------------------------------------------
    // 6. Accept the road plane
    // -------------------------------------------------

    if (fullInlierCount < MIN_FULL_INLIERS)
        return false;

    rp.normal = bestNormal;
    rp.d = bestD;

    return true;
}

// HELPER
std::vector<cv::Vec3d> matToVector(
    const cv::Mat_<cv::Vec3d> &pointCloud,
    const cv::Rect &roi,
    double minDepth,
    double maxDepth)
{
    std::vector<cv::Vec3d> points;

    // Limit ROI to actual matrix boundaries
    const cv::Rect bounds(
        0, 0, pointCloud.cols, pointCloud.rows);

    const cv::Rect validROI = roi & bounds;

    points.reserve(validROI.area());

    for (int y = validROI.y;
         y < validROI.y + validROI.height;
         ++y)
    {
        for (int x = validROI.x;
             x < validROI.x + validROI.width;
             ++x)
        {
            const cv::Vec3d &p = pointCloud(y, x);

            // Skip invalid reconstructed 3D points
            if (!std::isfinite(p[0]) ||
                !std::isfinite(p[1]) ||
                !std::isfinite(p[2]) ||
                p[2] < minDepth ||
                p[2] > maxDepth)
            {
                continue;
            }

            points.push_back(p);
        }
    }

    return points;
}
