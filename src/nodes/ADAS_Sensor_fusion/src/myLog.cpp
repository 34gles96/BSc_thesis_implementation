#include "myLog.hpp"

#include <rclcpp/rclcpp.hpp>

#include <fstream>
#include <iomanip>
#include <filesystem>
#include <mutex>
#include <iostream>

namespace myLOG
{

    std::ofstream gnss_log_file;
    std::ofstream ekf_log_file;

    std::mutex gnss_log_mutex;
    std::mutex ekf_log_mutex;

    double stampToMilliseconds(
        const builtin_interfaces::msg::Time &stamp)
    {
        return static_cast<double>(
                   rclcpp::Time(stamp).nanoseconds()) /
               1e6;
    }

    // ================================================================
    // GNSS LOGGING
    // ================================================================

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

    // ================================================================
    // EKF STATE LOGGING
    // ================================================================

    void initializeEkfLog()
    {
        std::filesystem::create_directories(
            "/ADC_27/logs");

        ekf_log_file.open(
            "/ADC_27/logs/ekf_state.csv",
            std::ios::out | std::ios::trunc);

        if (!ekf_log_file.is_open())
        {
            std::cerr
                << "Could not open EKF state log file."
                << std::endl;

            return;
        }

        ekf_log_file
            << "timestamp_ms,"
            << "E,N,U,"
            << "qw,qx,qy,qz,"
            << "bgx,bgy,bgz\n";

        ekf_log_file << std::setprecision(15);
    }

    void logEkfState(
        double timestamp_ms,
        const GeneratedEkfState &state)
    {
        if (!ekf_log_file.is_open())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(
            ekf_log_mutex);

        ekf_log_file
            << timestamp_ms << ","

            << state.p[0] << ","
            << state.p[1] << ","
            << state.p[2] << ","

            << state.q[0] << ","
            << state.q[1] << ","
            << state.q[2] << ","
            << state.q[3] << ","

            << state.bg[0] << ","
            << state.bg[1] << ","
            << state.bg[2]

            << "\n";
    }

    void closeEkfLog()
    {
        std::lock_guard<std::mutex> lock(
            ekf_log_mutex);

        if (ekf_log_file.is_open())
        {
            ekf_log_file.close();
        }
    }

}