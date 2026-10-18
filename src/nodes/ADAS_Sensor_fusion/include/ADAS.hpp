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

namespace
{

    std::ofstream gnss_log_file;
    std::mutex gnss_log_mutex;

    double stampToMilliseconds(
        const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(
                   rclcpp::Time(stamp).nanoseconds()) /
               1e6;
    }

    void initializeGnssLog()
    {
        std::filesystem::create_directories(
            "/ADC_27/logs");

        gnss_log_file.open(
            "/ADC_27/logs/gnss_lla.csv",
            std::ios::out | std::ios::trunc);

        if (!gnss_log_file.is_open())
        {
            std::cerr
                << "Could not open GNSS log file."
                << std::endl;

            return;
        }

        gnss_log_file
            << "timestamp_ms,"
            << "lat1,lon1,alt1,"
            << "lat2,lon2,alt2\n";

        gnss_log_file << std::setprecision(15);
    }

    void logGnssLla(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gnss1,
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gnss2)
    {
        if (!gnss_log_file.is_open())
        {
            return;
        }

        const double timestamp1_ms =
            stampToMilliseconds(
                gnss1->header.stamp);

        const double timestamp2_ms =
            stampToMilliseconds(
                gnss2->header.stamp);

        const double timestamp_ms =
            0.5 * (timestamp1_ms +
                   timestamp2_ms);

        std::lock_guard<std::mutex> lock(
            gnss_log_mutex);

        gnss_log_file
            << timestamp_ms << ","
            << gnss1->latitude << ","
            << gnss1->longitude << ","
            << gnss1->altitude << ","
            << gnss2->latitude << ","
            << gnss2->longitude << ","
            << gnss2->altitude
            << "\n";
    }

    void closeGnssLog()
    {
        std::lock_guard<std::mutex> lock(
            gnss_log_mutex);

        if (gnss_log_file.is_open())
        {
            gnss_log_file.close();
        }
    }

}