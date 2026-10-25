#pragma once

#include <vector>
#include <cmath>
#include <opencv2/core.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "desiredTrajectory.hpp"
#include "initializeCameraMounting.hpp"
#include "transformCameraToCoG.h"
#include "transformBodyToENU.h"

struct DesiredTrajectoryENU
{
    std::vector<cv::Point> uv_pixels;
    std::vector<cv::Vec3d> xyz_enu;
};

inline DesiredTrajectoryENU transformTrajectoryToENU(
    const DesiredTrajectory &trajectory_cam,
    const geometry_msgs::msg::PoseStamped &ekf_pose,
    const CamType::Transform4x4_Cam2Body &T_cam2body)
{
    DesiredTrajectoryENU result;

    if (trajectory_cam.uv_pixels.size() !=
        trajectory_cam.xyz_points.size())
        return result;

    // EKF CoG position in ENU
    const double p_cog_enu[3] = {
        ekf_pose.pose.position.x,
        ekf_pose.pose.position.y,
        ekf_pose.pose.position.z};

    // MATLAB quaternion format: [w, x, y, z]
    double q[4] = {
        ekf_pose.pose.orientation.w,
        ekf_pose.pose.orientation.x,
        ekf_pose.pose.orientation.y,
        ekf_pose.pose.orientation.z};

    result.uv_pixels.reserve(trajectory_cam.uv_pixels.size());
    result.xyz_enu.reserve(trajectory_cam.xyz_points.size());

    for (size_t i = 0; i < trajectory_cam.xyz_points.size(); ++i)
    {
        const cv::Vec3f &point = trajectory_cam.xyz_points[i];

        if (!std::isfinite(point[0]) ||
            !std::isfinite(point[1]) ||
            !std::isfinite(point[2]))
            continue;

        // Camera-frame point
        double p_camera[3] = {
            point[0],
            point[1],
            point[2]};

        // Camera -> Body (CoG)
        double p_body[3]{};

        transformCameraToCoG(
            p_camera,
            T_cam2body.data(),
            p_body);

        // Body -> ENU
        double p_enu[3]{};

        transformBodyToENU(
            p_body,
            p_cog_enu,
            q,
            p_enu);

        result.xyz_enu.emplace_back(
            p_enu[0],
            p_enu[1],
            p_enu[2]);

        result.uv_pixels.push_back(
            trajectory_cam.uv_pixels[i]);
    }

    return result;
}