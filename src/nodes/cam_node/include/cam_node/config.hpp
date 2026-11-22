// config.hpp
#pragma once

#include <algorithm>
#include <cmath>
#include "readConfig.hpp"

namespace config
{
    inline double PREDICTED_HORIZON = 20; // modified, initializated in via updateTrajectoryBounds  in running
    inline double MIN_TRAJECTORY_Z = 4;   // modified in running

    // inline constexpr double img_in_stream_vertical_cut_top = 0.55;
    inline constexpr double img_in_stream_vertical_cut_top = 0.55;
    // inline constexpr double img_in_stream_vertical_cut_bottom = 0.80;
    inline constexpr double img_in_stream_vertical_cut_bottom = 0.9; // 0.9
    // inline constexpr double img_in_stream_vertical_cut_bottom = 0.75;

    inline void updateTrajectoryBounds(
        const CameraParams &cam,
        int num_disparities,
        double cam_height_m = 1.05,
        double max_lookahead_m = 25.0,
        double max_depth_err_m = 0.20,
        double disp_precision = 0.1) // max allowed disparitx error in pixels dimension
    {
        const double f = cam.fy;
        const double cy = cam.cy;
        const double B = std::abs(cam.baselink);

        const double v_top = img_in_stream_vertical_cut_top * static_cast<double>(cam.height);
        const double v_bottom = img_in_stream_vertical_cut_bottom * static_cast<double>(cam.height);

        const double z_stereo_min = (num_disparities > 0 && B > 1e-6)
                                        ? (f * B) / static_cast<double>(num_disparities)
                                        : 2.0;

        const double z_frustum_min = (v_bottom > cy + 1.0)
                                         ? (f * cam_height_m) / (v_bottom - cy)
                                         : z_stereo_min;

        const double z_frustum_max = (v_top > cy + 1.0)
                                         ? (f * cam_height_m) / (v_top - cy)
                                         : max_lookahead_m;

        const double z_noise_max = (disp_precision > 0.0 && B > 1e-6)
                                       ? std::sqrt((f * B * max_depth_err_m) / disp_precision)
                                       : max_lookahead_m;

        // MIN_TRAJECTORY_Z = std::max(z_stereo_min, z_frustum_min);
        double offsetFromMinHorizont = 3;
        // PREDICTED_HORIZON = std::min({z_frustum_max, z_noise_max, max_lookahead_m});
        // PREDICTED_HORIZON = std::max(offsetFromMinHorizont, PREDICTED_HORIZON);
    }
}