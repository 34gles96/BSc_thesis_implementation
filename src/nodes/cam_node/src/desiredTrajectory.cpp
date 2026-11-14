#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

#include "desiredTrajectory.hpp"
#include "fitLanePoly.hpp"
#include "config.hpp"

// Draw trajectory and lane line on the original image frame
DesiredTrajectory drawDesiredTrajectoryOnImage(
    cv::Mat &original_bgr_frame,
    double a2, double a1, double a0, // Polynomial coeffs: X(Z) = a2*Z^2 + a1*Z + a0
    const RoadPlane &road_plane,     // Camera height relative to road (in camera frame, Y is down)
    double &lane_width,              // e.g., 3.5 meters
    bool is_left_lane,               // true if left line, false if right line
    const CameraParams &cam,
    struct LanePolynomial lanepoly,
    double max_detected_z)
{

    if (lane_width < 9)
    {
        lane_width = 12;
    }

    // for return type
    DesiredTrajectory result;
    float nan = std::numeric_limits<float>::quiet_NaN();

    result.xyz_map = cv::Mat_<cv::Vec3f>(
        original_bgr_frame.size(),
        cv::Vec3f(nan, nan, nan));

    std::vector<cv::Point> lane_2d_pixels;
    std::vector<cv::Point> trajectory_2d_pixels;

    // Evaluate depths from 3 meters out to 25 meters ahead
    // --- 2. Clamp the effective horizon to never exceed real data ---
    double max_allowed_horizon = 10.0; // <-- SET YOUR DESIRED MAX LENGTH IN METERS HERE (e.g., 8m or 10m)

    double effective_horizon = std::min({static_cast<double>(config::PREDICTED_HORIZON),
                                         max_detected_z,
                                         max_allowed_horizon});
    double effective_min_trajectory = std::min(effective_horizon, config::MIN_TRAJECTORY_Z);
    for (double Z = effective_min_trajectory; Z <= effective_horizon; Z += 0.5)
    {

        // 1. Calculate 3D position on the detected lane line
        double X_line = a2 * Z * Z + a1 * Z + a0;

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

        // 3. Compute 3D Inward Unit Normal Vector (-Tz, 0, Tx)
        double Nx = -Tz;
        double Nz = Tx;

        // Ensure normal points INWARD toward road center (X = 0)
        // Left lane -> Normal must point Right (+X)
        // Right lane -> Normal must point Left (-X)
        if (is_left_lane && Nx < 0)
        {
            Nx = -Nx;
            Nz = -Nz;
        }
        else if (!is_left_lane && Nx > 0)
        {
            Nx = -Nx;
            Nz = -Nz;
        }

        // 4. Calculate 3D Target Centerline Point (P_line + (w/2) * N)
        double half_width = lane_width / 2.0;
        double X_center = X_line + half_width * Nx;
        double Y_center = Y_line; // Stays on the ground plane
        double Z_center = Z + half_width * Nz;

        // 5. Project 3D Lane Point to 2D Image Pixels
        int u_line = static_cast<int>((cam.fx * X_line) / Z + cam.cx);
        int v_line = static_cast<int>((cam.fy * Y_line) / Z + cam.cy);

        // 6. Project 3D Centerline Point to 2D Image Pixels
        int u_center = static_cast<int>((cam.fx * X_center) / Z_center + cam.cx);
        int v_center = static_cast<int>((cam.fy * Y_center) / Z_center + cam.cy);

        // Bounds check before adding to draw list
        if (u_line >= 0 && u_line < original_bgr_frame.cols && v_line >= 0 && v_line < original_bgr_frame.rows)
        {
            lane_2d_pixels.push_back(cv::Point(u_line, v_line));
        }

        if (u_center >= 0 && u_center < original_bgr_frame.cols &&
            v_center >= 0 && v_center < original_bgr_frame.rows)
        {
            trajectory_2d_pixels.emplace_back(u_center, v_center);

            // Store one XYZ point per trajectory pixel
            if (!std::isfinite(result.xyz_map(v_center, u_center)[2]))
            {
                result.uv_pixels.emplace_back(u_center, v_center);

                cv::Vec3f XYZ(
                    static_cast<float>(X_center),
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

    // 8. Draw the Desired Centerline Trajectory (Green - Thicker)
    for (size_t i = 1; i < trajectory_2d_pixels.size(); ++i)
    {
        cv::line(original_bgr_frame, trajectory_2d_pixels[i - 1], trajectory_2d_pixels[i], cv::Scalar(0, 255, 0), 4);
        cv::circle(original_bgr_frame, trajectory_2d_pixels[i], 3, cv::Scalar(0, 255, 255), -1); // Yellow waypoints
    }
    return result;
}