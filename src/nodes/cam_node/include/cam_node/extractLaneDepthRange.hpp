#include <opencv2/opencv.hpp>
#include <vector>
#include <limits>
#include <algorithm>

struct DepthRange
{
    double min_z;
    double max_z;
};

inline DepthRange extractLaneDepthRange(
    const std::vector<cv::Point> &white_line_pts,
    const cv::Mat &point_cloud_3D_,
    double default_min = 3.0,
    double default_max = 25.0)
{
    double min_z = std::numeric_limits<double>::max();
    double max_z = std::numeric_limits<double>::lowest();

    for (const auto &pt : white_line_pts)
    {
        // Ensure pixel coordinates are within point cloud bounds
        if (pt.x >= 0 && pt.x < point_cloud_3D_.cols && pt.y >= 0 && pt.y < point_cloud_3D_.rows)
        {
            // Assuming point cloud is stored as CV_32FC3 where index [2] is Z (forward depth)
            cv::Vec3f p3d = point_cloud_3D_.at<cv::Vec3f>(pt.y, pt.x);

            if (std::isfinite(p3d[2]) && p3d[2] > 0.0)
            {
                min_z = std::min(min_z, static_cast<double>(p3d[2]));
                max_z = std::max(max_z, static_cast<double>(p3d[2]));
            }
        }
    }

    // Fallback defaults if no valid points were found in the frame
    if (min_z == std::numeric_limits<double>::max())
        min_z = default_min;
    if (max_z == std::numeric_limits<double>::lowest())
        max_z = default_max;

    return {min_z, max_z};
}