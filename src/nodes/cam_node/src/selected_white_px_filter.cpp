#include "vector"
#include "map"
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "detectObstacles.hpp"
#include <optional>
#include "selected_white_px_filter.hpp"

std::optional<std::vector<cv::Point>> filter_out_lat_extenison_px(
    const cv::Mat &XYZ_img,
    const std::vector<cv::Point> &white_px_line,
    const double depth_bin_size_meter,
    const CameraParams &cam,
    const RoadPlane &road_plane)
{

    std::map<int, std::vector<double>> depth_binVec;

    std::vector<cv::Point> white_px_line_filter;
    white_px_line_filter.reserve(white_px_line.size());

    for (const cv::Point2d &pixel : white_px_line)
    {

        // safety guard
        if (pixel.x < 0 || pixel.x >= XYZ_img.cols ||
            pixel.y < 0 || pixel.y >= XYZ_img.rows)
        {
            continue;
        }

        cv::Vec3f XYZ = XYZ_img.at<cv::Vec3f>(pixel.y, pixel.x);
        const double X = XYZ[0];
        const double Z = XYZ[2];

        // trasnfroming stream of white_px_line into XYZ corresponding w depthbins //  Grouping  extracted white pixels by depth
        const int binIndex =
            static_cast<int>(std::floor(Z / depth_bin_size_meter));

        depth_binVec[binIndex].push_back(X);
    }

    // filter out the laeral exnenion of X coordinate using average operation

    std::map<int, double> depth_bin_average_tmp;
    auto avg = [](const std::vector<double> &in) -> double
    {
        double X = 0;
        for (const auto &i : in)
        {
            X += i;
        }
        X /= static_cast<double>(in.size());
        return X;
    };

    for (const auto &it : depth_binVec)
    {
        depth_bin_average_tmp[it.first] = avg(it.second);
    }

    /// ============= FILTER LATERAL EXTENSION =============

    for (const auto &it : depth_bin_average_tmp)
    {

        // inverse trasforming : depthibin -> Z

        const double Zcenter =
            (it.first + 0.5) * depth_bin_size_meter;

        const double averageX = it.second;

        // Road plane: AX + BY + CZ + D = 0
        const double A = road_plane.normal[0];
        const double B = road_plane.normal[1];
        const double C = road_plane.normal[2];
        const double D = road_plane.d;

        if (!std::isfinite(A) ||
            !std::isfinite(B) ||
            !std::isfinite(C) ||
            !std::isfinite(D) ||
            std::abs(B) < 1e-6)
            return std::nullopt;

        // Reconstruct Y using the averaged X
        const double Y_value =
            (-A * averageX - C * Zcenter - D) / B;

        // Project representative 3D point into image
        const double u =
            cam.fx * averageX / Zcenter + cam.cx;

        const double v =
            cam.fy * Y_value / Zcenter + cam.cy;

        if (!std::isfinite(u) || !std::isfinite(v))
            continue;

        const int u_pixel = static_cast<int>(std::round(u));
        const int v_pixel = static_cast<int>(std::round(v));

        if (u_pixel >= 0 && u_pixel < XYZ_img.cols &&
            v_pixel >= 0 && v_pixel < XYZ_img.rows)
        {
            white_px_line_filter.emplace_back(
                u_pixel, v_pixel);
        }
    }

    return white_px_line_filter;
}