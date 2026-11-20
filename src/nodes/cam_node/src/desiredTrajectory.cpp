#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

#include "desiredTrajectory.hpp"
#include "fitLanePoly.hpp"
#include "config.hpp"

// Draw trajectory and lane line on the original image frame
DesiredTrajectory drawDesiredTrajectoryOnImage(
    cv::Mat &original_bgr_frame,
    const RoadPlane &road_plane, // Camera height relative to road (in camera frame, Y is down)
    double &lane_width,          // e.g., 3.5 meters
    const CameraParams &cam,
    const std::vector<LanePolynomial> &lanepoly_vec, // contains  Polynomial coeffs: X(Z) = a2*Z^2 + a1*Z + a0 for best and 2ndBest selected road boandary
    double max_detected_z)
{

    if (lane_width < 9)
    {
        lane_width = 12;
    }

    const auto &[a2, a1, a0, isValid, is_left_lane, is_road_width_Valid] = lanepoly_vec[0];
    const auto &[a2_2ndBest, a1_2ndBest, a0_2ndBest, isValid_2ndBest, is_left_lane_2ndBest, is_road_width_Valid_2ndBest] = lanepoly_vec[1];
    // for return type
    DesiredTrajectory result;
    float nan = std::numeric_limits<float>::quiet_NaN();

    result.xyz_map = cv::Mat_<cv::Vec3f>(
        original_bgr_frame.size(),
        cv::Vec3f(nan, nan, nan));

    std::vector<cv::Point> lane_2d_pixels; // for debugging/displax
    // 2nd best
    std::vector<cv::Point> lane_2d_pixels_2ndBest; // for debugging/displax
    std::vector<cv::Point> trajectory_2d_pixels;

    // Evaluate depths from 3 meters out to 25 meters ahead
    // --- 2. Clamp the effective horizon to never exceed real data ---
    // double effective_horizon = std::min(static_cast<double>(config::PREDICTED_HORIZON), max_detected_z);
    double effective_horizon = static_cast<double>(config::PREDICTED_HORIZON);
    double effective_min_trajectory = static_cast<double>(config::MIN_TRAJECTORY_Z);
    for (double Z = effective_min_trajectory; Z <= effective_horizon; Z += 0.05)
    {
        // 1. Calculate 3D position on the detected lane line
        double X_line = a2 * Z * Z + a1 * Z + a0;
        double X_line_2ndBest = a2_2ndBest * Z * Z + a1_2ndBest * Z + a0_2ndBest;

        // TODO construct X_line_2ndBest

        // <-- CHANGED: DYNAMIC Y CALCULATION --> // Camera position froum ground in camera frame
        // Solve AX + BY + CZ + D = 0 for Y
        double A = road_plane.normal[0];
        double B = road_plane.normal[1];
        double C = road_plane.normal[2];
        double D = road_plane.d;

        // Prevent division by zero if the plane is perfectly vertical (should be caught by your RANSAC, but good for safety)
        if (std::abs(B) < 1e-6)
            B = -1.0;

        double Y_line = (-A * X_line - C * Z - D) / B;

        // ============================== // CALCULATE ROAD WIDTH // ===============================================

        // =========================================================================================================

        // 2. Compute 3D Unit Tangent Vector along the line: T = [dX/dZ, dY/dZ, 1]
        double dX_dZ = 2.0 * a2 * Z + a1;
        double T_norm = std::sqrt(dX_dZ * dX_dZ + 1.0);
        double Tx = dX_dZ / T_norm;
        double Tz = 1.0 / T_norm;

        // 2nd best
        double dX_dZ_2ndBest = 2.0 * a2_2ndBest * Z + a1_2ndBest;
        double T_norm_2ndBest = std::sqrt(dX_dZ_2ndBest * dX_dZ_2ndBest + 1.0);
        double Tx_2ndBest = dX_dZ_2ndBest / T_norm_2ndBest;
        double Tz_2ndBest = 1.0 / T_norm_2ndBest;

        // 3. Compute Inward Unit Normal Vector pointing toward road center
        // Left boundary  (is_left_lane == true)  -> normal points Right (+X): [ +Tz, -Tx ]
        // Right boundary (is_left_lane == false) -> normal points Left  (-X): [ -Tz, +Tx ]
        double sign = is_left_lane ? 1.0 : -1.0;
        double Nx = sign * Tz;
        double Nz = -sign * Tx;

        // TODO: introduce some distasnce between red lines.. makineg a more striclty contraint for using 2 red lines
        //  2nd best
        double sign_2ndBest = (!is_left_lane) ? 1.0 : -1.0;
        double Nx_2ndBest = sign_2ndBest * Tz_2ndBest;
        double Nz_2ndBest = -sign_2ndBest * Tx_2ndBest;

        // 4. Calculate 3D Target Centerline Point (P_line + (w/2) * N)
        double half_width = lane_width / 2.0;
        double X_center = X_line + half_width * Nx;
        double Y_center = Y_line; // Stays on the ground plane
        double Z_center = Z + half_width * Nz;

        // 2nd Best
        double X_center_2ndBest = X_line_2ndBest + half_width * Nx_2ndBest;

        // CALUCLATE EFFECTIVE X_LINE THAT DESCRIBES THE LATERAL POSITION OF DESIRED TRAJECTORY: mean of ..:
        double X_center_effective;
        if (isValid_2ndBest)
        {
            // X_center_effective = (X_center + X_center_2ndBest) / 2;
            X_center_effective = 0.5 * (X_line + X_line_2ndBest);
        }
        else
        {
            X_center_effective = X_center;
        }

        // 5. Project 3D Lane Point to 2D Image Pixels - Reverese transfrming XZY into (u,v) pixelse for debgginf purpose/display
        int u_line = static_cast<int>((cam.fx * X_line) / Z + cam.cx);
        int v_line = static_cast<int>((cam.fy * Y_line) / Z + cam.cy);

        // 2nd Best
        int u_line_2nBest = static_cast<int>((cam.fx * X_line_2ndBest) / Z + cam.cx);

        // 6. Project 3D Centerline Point to 2D Image Pixels
        int u_center = static_cast<int>((cam.fx * X_center_effective) / Z_center + cam.cx);
        int v_center = static_cast<int>((cam.fy * Y_line) / Z_center + cam.cy);

        // Bounds check before adding to draw list
        if (u_line >= 0 && u_line < original_bgr_frame.cols && v_line >= 0 && v_line < original_bgr_frame.rows)
        {
            lane_2d_pixels.push_back(cv::Point(u_line, v_line));
        }

        // 2nd best
        if (u_line_2nBest >= 0 && u_line_2nBest < original_bgr_frame.cols && v_line >= 0 && v_line < original_bgr_frame.rows)
            lane_2d_pixels_2ndBest.push_back(cv::Point(u_line_2nBest, v_line));

        if (u_center >= 0 && u_center < original_bgr_frame.cols &&
            v_center >= 0 && v_center < original_bgr_frame.rows)
        {
            trajectory_2d_pixels.emplace_back(u_center, v_center);

            // Store one XYZ point per trajectory pixel
            if (!std::isfinite(result.xyz_map(v_center, u_center)[2]))
            {
                result.uv_pixels.emplace_back(u_center, v_center);

                cv::Vec3f XYZ(
                    static_cast<float>(X_center_effective),
                    static_cast<float>(Y_center),
                    static_cast<float>(Z_center));

                result.xyz_points.push_back(XYZ);

                result.xyz_map(v_center, u_center) = XYZ;
            }
        }
    }

    // 7. Draw the Detected Lane Line (Red)
    for (size_t i = 1; i < lane_2d_pixels.size(); ++i)
    {
        cv::line(original_bgr_frame, lane_2d_pixels[i - 1], lane_2d_pixels[i], cv::Scalar(0, 0, 255), 2);
    }
    for (size_t i = 1; i < lane_2d_pixels_2ndBest.size(); ++i)
    {
        cv::line(original_bgr_frame, lane_2d_pixels_2ndBest[i - 1], lane_2d_pixels_2ndBest[i], cv::Scalar(0, 0, 255), 2);
    }

    // 8. Draw the Desired Centerline Trajectory (Green - Thicker)
    for (size_t i = 1; i < trajectory_2d_pixels.size(); ++i)
    {
        cv::line(original_bgr_frame, trajectory_2d_pixels[i - 1], trajectory_2d_pixels[i], cv::Scalar(0, 255, 0), 4);
        cv::circle(original_bgr_frame, trajectory_2d_pixels[i], 3, cv::Scalar(0, 255, 255), -1); // Yellow waypoints
    }
    return result;
}