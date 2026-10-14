#pragma once
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"

struct EkfState
{
    Eigen::Vector3d p;
    Eigen::Vector4d q; // [qw qx qy qz]
    Eigen::Vector3d bg;

    Eigen::Matrix<double, 10, 10> P;

    rclcpp::Time stamp;
};

// sensor integration layer
struct DualGnssData
{
    Eigen::Vector3d p1_enu;
    Eigen::Vector3d p2_enu;

    rclcpp::Time stamp;

    bool isUsed;
};

struct ImuData
{
    Eigen::Vector3d acceleration;     // [ax, ay, az]  [m/s^2]
    Eigen::Vector3d angular_velocity; // [wx, wy, wz]  [rad/s]

    rclcpp::Time stamp;
};

struct SteeringData
{
    double angle; // steering angle [rad]
    rclcpp::Time stamp;
};