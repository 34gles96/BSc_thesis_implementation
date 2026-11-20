#pragma once

#include "vector"
#include <opencv2/opencv.hpp>
#include "config.hpp"
#include "detectObstacles.hpp"
#include <optional>
#include "readConfig.hpp"
#include "detectObstacles.hpp"

std::optional<std::vector<cv::Point>> filter_out_lat_extenison_px(
    const cv::Mat &XYZ_img,
    const std::vector<cv::Point> &white_px_line,
    const double depth_bin_size_meter,
    const CameraParams &cam,
    const RoadPlane &road_plane);