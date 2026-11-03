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

bool detectRoad(struct RoadPlane &rp,
                const cv::Mat_<cv::Vec3d> &XYZ_cam)
{

    constexpr double minRoadDepth = 8.0;
    constexpr double maxRoadDepth = 15.0;

    cv::Rect roadROI(
        0,
        XYZ_cam.rows / 2,
        XYZ_cam.cols,
        XYZ_cam.rows - XYZ_cam.rows / 2);

    std::vector<cv::Vec3d> pointCloud = matToVector(
        XYZ_cam,
        roadROI,
        minRoadDepth,
        maxRoadDepth);

    if (pointCloud.size() < 3)
        return false;

    std::mt19937 rng(42);                                                          // deterministic pseudosquencte of integers
    std::uniform_int_distribution<size_t> pointSelector(0, pointCloud.size() - 1); // uniformly distribtued seqence of det seq of integers

    constexpr int iterations = 350;
    constexpr double threshold_road = 0.15;
    const cv::Vec3d up_cam(0.0, -1.0, 0.0);
    int best_inlierCount = 0;

    for (size_t i = 0; i < iterations; i++)
    {

        // chosing from unifromly distrubited generated point of image
        const size_t a = pointSelector(rng);
        const size_t b = pointSelector(rng);
        const size_t c = pointSelector(rng); // purpose of const?

        if (a == b || a == c || b == c)
            continue;

        const cv::Vec3d &p1 = pointCloud[a];
        const cv::Vec3d &p2 = pointCloud[b];
        const cv::Vec3d &p3 = pointCloud[c];

        cv::Vec3d v1 = p1 - p2;
        cv::Vec3d v2 = p1 - p3;

        cv::Vec3d norm_vec = v1.cross(v2);
        double norm_magn = cv::norm(norm_vec);

        // safaty guard
        if (norm_magn < 1e-9)
            continue;

        norm_vec /= norm_magn;

        // orient norm vector
        if (norm_vec.dot(up_cam) < 0)
            norm_vec = -norm_vec;

        // Reject approximately vertical surfaces
        if (norm_vec.dot(up_cam) < 0.80)
            continue;

        //  Calculate plane coefficient d
        double d = -norm_vec.dot(p1);

        int inlierCount = 0;
        for (const auto &point : pointCloud)
        {
            double vertical_height_naiv = std::abs(
                norm_vec.dot(point) + d);

            if (vertical_height_naiv < threshold_road)
                ++inlierCount;
        }

        if (inlierCount > best_inlierCount)
        {
            best_inlierCount = inlierCount;

            rp.normal = norm_vec;
            rp.d = d;
        }
    }

    return best_inlierCount >= 100;
}

void detectObstacles(const cv::Mat_<cv::Vec3d> &XYZ_cam,
                     const RoadPlane &roadplane,
                     const double obstacleTreshold,
                     cv::Mat1d &heightMap,
                     cv::Mat1b &ObstacleMast,
                     const cv::Vec3d &upDirecton)
{

    heightMap = cv::Mat1d(
        XYZ_cam.size(),
        std::numeric_limits<double>::quiet_NaN());

    ObstacleMast = cv::Mat1b::zeros(XYZ_cam.size());

    // Normalize upward direction
    const double upNorm = cv::norm(upDirecton);

    if (upNorm < 1e-9)
        throw std::runtime_error("Invalid upward vector");

    const cv::Vec3d unitUp = upDirecton / upNorm;

    const double denominator = roadplane.normal.dot(unitUp); // due to expression of calc of vetical height

    if (std::abs(denominator) < 1e-6) // safety guard
        throw std::runtime_error("Invalid road-plane orientation");

    // Examine every image pixel
    for (int y = 0; y < XYZ_cam.rows; ++y)
    {
        for (int x = 0; x < XYZ_cam.cols; ++x)
        {
            const cv::Vec3d &point = XYZ_cam(y, x);

            // Reject invalid reconstructed points
            if (!std::isfinite(point[0]) ||
                !std::isfinite(point[1]) ||
                !std::isfinite(point[2]) ||
                point[2] <= 0.0)
            {
                continue;
            }

            constexpr double minObstacleDepth = 0.5;
            constexpr double maxObstacleDepth = 30.0;

            if (point[2] < minObstacleDepth ||
                point[2] > maxObstacleDepth)
            {
                continue;
            }

            // Vertical height above road in meters
            const double height =
                (roadplane.normal.dot(point) + roadplane.d) / denominator;

            heightMap(y, x) = height;

            // Classify potential obstacle pixels if (height > heightThreshold)

            if (height > obstacleTreshold)
            {
                ObstacleMast(y, x) = 255;
            }
        }
    }
}

void drawDetectedObstacles(
    const cv::Mat1b &obstacleMask,
    cv::Mat &leftImage)
{
    // safety guard
    if (obstacleMask.empty() ||
        leftImage.empty() ||
        obstacleMask.size() != leftImage.size())
        return;

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;

    int numComponents = cv::connectedComponentsWithStats(
        obstacleMask,
        labels,
        stats,
        centroids,
        8,
        CV_32S);

    // ============= CONFIG TYPES ===============
    constexpr int minArea = 350; // Increase to filter more noise
    constexpr int maxBoxes = 15; // Maximum displayed boxes
    constexpr int minWidth = 9;
    constexpr int minHeight = 9;

    struct Region
    {
        int id;
        cv::Rect box;
        int area;
    };

    std::vector<Region> regions;

    // ===================================

    // ===== PROCESING STATS OF OBSTACLES ======
    // Start at 1 because label 0 is background.
    for (int id = 1; id < numComponents; ++id)
    {
        int x = stats.at<int>(id, cv::CC_STAT_LEFT);
        int y = stats.at<int>(id, cv::CC_STAT_TOP);

        int width =
            stats.at<int>(id, cv::CC_STAT_WIDTH);

        int height =
            stats.at<int>(id, cv::CC_STAT_HEIGHT);

        int area =
            stats.at<int>(id, cv::CC_STAT_AREA);

        // Ignore very small regions
        if (area < minArea ||
            width < minWidth ||
            height < minHeight)
            continue;

        // Ignore regions entirely above the image midpoint
        if (y + height < leftImage.rows / 2)
            continue;

        regions.push_back({id,
                           cv::Rect(x, y, width, height),
                           area});
    }

    // =====================================================================

    // Sort regions by number of detected obstacle pixels
    std::sort(
        regions.begin(),
        regions.end(),
        [](const Region &a, const Region &b)
        {
            return a.area > b.area;
        });

    // Color-blind-friendly palette (OpenCV uses BGR)
    const cv::Scalar boxColor(178, 114, 0); // Blue
    const cv::Scalar labelBg(66, 228, 240); // Yellow
    const cv::Scalar textColor(0, 0, 0);    // Black

    // Draw only the largest regions
    int count = std::min(
        maxBoxes,
        static_cast<int>(regions.size()));

    for (int i = 0; i < count; ++i)
    {
        const Region &region = regions[i];
        const cv::Rect &boundingBox = region.box;

        // ===============.  Draw bounding box with dark outer border ======

        // Draw bounding box
        cv::rectangle(
            leftImage,
            boundingBox,
            cv::Scalar(178, 114, 0), // blue
            2);

        // Simple label text
        std::string label =
            "# " + std::to_string(region.id);

        const int font = cv::FONT_HERSHEY_SIMPLEX;
        const double fontScale = 0.45;
        const int thickness = 1;
        const int padding = 3;

        int baseline = 0;

        cv::Size textSize = cv::getTextSize(
            label,
            font,
            fontScale,
            thickness,
            &baseline);

        int labelW = textSize.width + 2 * padding;
        int labelH = textSize.height + baseline + 2 * padding;

        // Put label above box if possible, otherwise slightly inside
        int labelX = boundingBox.x;
        int labelY = boundingBox.y - labelH - 2;

        if (labelY < 0)
            labelY = boundingBox.y + 2;

        labelX = std::clamp(
            labelX,
            0,
            leftImage.cols - labelW);

        labelY = std::clamp(
            labelY,
            0,
            leftImage.rows - labelH);

        // Light simple background
        cv::rectangle(
            leftImage,
            cv::Rect(labelX, labelY, labelW, labelH),
            cv::Scalar(230, 230, 230), // light gray
            cv::FILLED);

        // Thin border around label
        cv::rectangle(
            leftImage,
            cv::Rect(labelX, labelY, labelW, labelH),
            cv::Scalar(80, 80, 80),
            1);

        // Normal black text
        cv::putText(
            leftImage,
            label,
            cv::Point(
                labelX + padding,
                labelY + padding + textSize.height),
            font,
            fontScale,
            cv::Scalar(0, 0, 0),
            thickness,
            cv::LINE_AA);

        std::cout
            << "Region " << region.id
            << " | Area: " << region.area << " pixels"
            << " | Position: (" << boundingBox.x
            << ", " << boundingBox.y << ")"
            << "\n";
    }
}

void filterObstaclesToLane(
    const cv::Mat_<cv::Vec3d> &XYZ_cam,
    cv::Mat1b &obstacleMask,
    const LanePolynomial &roadPoly,
    double laneWidth)
{
    CV_Assert(XYZ_cam.size() == obstacleMask.size());
    CV_Assert(laneWidth > 0.0);

    for (int y = 0; y < XYZ_cam.rows; ++y)
    {
        for (int x = 0; x < XYZ_cam.cols; ++x)
        {
            // Only process previously detected obstacle pixels
            if (obstacleMask(y, x) == 0)
                continue;

            const cv::Vec3d &point = XYZ_cam(y, x);

            const double X = point[0];
            const double Z = point[2];

            if (!std::isfinite(X) ||
                !std::isfinite(Z) ||
                Z <= 0.0)
            {
                obstacleMask(y, x) = 0;
                continue;
            }

            // Evaluate white road-line polynomial at depth Z
            const double whiteLineX =
                roadPoly.a2 * Z * Z + roadPoly.a1 * Z + roadPoly.a0;

            if (!std::isfinite(whiteLineX))
            {
                obstacleMask(y, x) = 0;
                continue;
            }

            double minX;
            double maxX;

            if (!roadPoly.is_left_lane)
            {
                minX = whiteLineX - laneWidth;
                maxX = whiteLineX;
            }
            else
            {
                minX = whiteLineX;
                maxX = whiteLineX + laneWidth;
            }

            // Reject detected obstacle pixels outside the lane
            if (X < minX || X > maxX)
            {
                obstacleMask(y, x) = 0;
            }
        }
    }
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
