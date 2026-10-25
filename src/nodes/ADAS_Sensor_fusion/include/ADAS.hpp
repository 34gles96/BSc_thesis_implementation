#pragma once
#include <Eigen/Dense>
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <iomanip>
#include <filesystem>
#include <mutex>
#include <sensor_msgs/msg/nav_sat_fix.hpp>

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
    Eigen::Vector3d lla1;
    Eigen::Vector3d lla2;

    double hAcc1;
    double vAcc1;
    double hAcc2;
    double vAcc2;

    double timestamp_ms;

    bool isUsed{true};
};

struct ImuData
{
    Eigen::Vector3d acceleration;     // [ax, ay, az]  [m/s^2]
    Eigen::Vector3d angular_velocity; // [wx, wy, wz]  [rad/s]

    rclcpp::Time stamp;
};

struct SteeringData
{
    double angle;       // steering angle [rad]
    rclcpp::Time stamp; // it s ignored bcs EKFa always uses the latest avaiable steering data
};

struct VelocityData
{
    double velocity{0.0};
};

