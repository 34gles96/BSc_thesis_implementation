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

namespace ekfReport
{

    std::ofstream report;
    std::mutex report_mutex;

    // ================================================================
    // EKF INPUT LOG INITIALIZATION
    // ================================================================

    void initializeReport()
    {
        std::filesystem::create_directories(
            "/ADC_27/logs");

        // ================================================================
        // START GROUND TRUTH ROS BAG
        // ================================================================

        const int bag_result =
            std::system(
                "/ADC_27/scripts/ekf_rosbag.sh start");

        if (bag_result != 0)
        {
            std::cerr
                << "Failed to start EKF ground-truth rosbag."
                << std::endl;
        }

        report.open(
            "/ADC_27/logs/ekf_input.csv",
            std::ios::out | std::ios::trunc);

        if (!report.is_open())
        {
            std::cerr
                << "Failed to open EKF input log file."
                << std::endl;

            return;
        }

        report << std::setprecision(17);

        // ============================================================
        // HEADER
        // ============================================================

        report
            // ========================================================
            // IMU
            // ========================================================

            << "imu_timestamp_ms,"
            << "imu_valid,"

            << "gyro_x,gyro_y,gyro_z,"

            << "gyro_cov_00,gyro_cov_01,gyro_cov_02,"
            << "gyro_cov_10,gyro_cov_11,gyro_cov_12,"
            << "gyro_cov_20,gyro_cov_21,gyro_cov_22,"

            << "accel_x,accel_y,accel_z,"

            << "accel_cov_00,accel_cov_01,accel_cov_02,"
            << "accel_cov_10,accel_cov_11,accel_cov_12,"
            << "accel_cov_20,accel_cov_21,accel_cov_22,"

            << "mount_x_x,mount_x_y,mount_x_z,"
            << "mount_y_x,mount_y_y,mount_y_z,"
            << "mount_z_x,mount_z_y,mount_z_z,"

            // ========================================================
            // GNSS
            // ========================================================

            << "gnss_valid,"
            << "gnss_timestamp_ms,"

            << "lla1_lat,lla1_lon,lla1_alt,"
            << "lla2_lat,lla2_lon,lla2_alt,"

            << "hAcc1,vAcc1,"
            << "hAcc2,vAcc2,"

            << "position1_enu_x,position1_enu_y,position1_enu_z,"
            << "position2_enu_x,position2_enu_y,position2_enu_z,"

            << "position_enu_x,position_enu_y,position_enu_z,"

            << "baseline_enu_x,baseline_enu_y,baseline_enu_z,"

            << "baseline_body_x,baseline_body_y,baseline_body_z,"

            // ========================================================
            // VEHICLE
            // ========================================================

            << "vehicle_velocity,"
            << "vehicle_steering_angle,"
            << "vehicle_wheelbase"

            << "\n";
    }

    // ================================================================
    // LOG INPUT OF ONE EKF_func CALL
    // ================================================================

    void reportEKF_in(
        const GeneratedImuData &imu,
        const GeneratedGnssData &gnss,
        const GeneratedVehicleData &vehicle)
    {
        if (!report.is_open())
        {
            return;
        }

        std::lock_guard<std::mutex> lock(
            report_mutex);

        // ============================================================
        // IMU
        // ============================================================

        report
            << imu.timestamp << ","
            << static_cast<int>(imu.valid) << ","

            << imu.gyro[0] << ","
            << imu.gyro[1] << ","
            << imu.gyro[2] << ",";

        // ============================================================
        // IMU GYRO COVARIANCE
        //
        // MATLAB generated arrays are column-major:
        //
        // index = row + 3 * col
        //
        // CSV is written in normal matrix row order.
        // ============================================================

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                report
                    << imu.gyro_covariance[row + 3 * col]
                    << ",";
            }
        }

        // ============================================================
        // IMU ACCELEROMETER
        // ============================================================

        report
            << imu.accel[0] << ","
            << imu.accel[1] << ","
            << imu.accel[2] << ",";

        // ============================================================
        // IMU ACCELEROMETER COVARIANCE
        //
        // MATLAB generated arrays are column-major:
        //
        // index = row + 3 * col
        //
        // CSV is written in normal matrix row order.
        // ============================================================

        for (int row = 0; row < 3; ++row)
        {
            for (int col = 0; col < 3; ++col)
            {
                report
                    << imu.accel_covariance[row + 3 * col]
                    << ",";
            }
        }

        // ============================================================
        // IMU MOUNTING
        // ============================================================

        report
            << imu.mounting.x_sensor_body[0] << ","
            << imu.mounting.x_sensor_body[1] << ","
            << imu.mounting.x_sensor_body[2] << ","

            << imu.mounting.y_sensor_body[0] << ","
            << imu.mounting.y_sensor_body[1] << ","
            << imu.mounting.y_sensor_body[2] << ","

            << imu.mounting.z_sensor_body[0] << ","
            << imu.mounting.z_sensor_body[1] << ","
            << imu.mounting.z_sensor_body[2] << ",";

        // ============================================================
        // GNSS
        // ============================================================

        report
            << static_cast<int>(gnss.valid) << ","
            << gnss.timestamp << ","

            << gnss.lla1[0] << ","
            << gnss.lla1[1] << ","
            << gnss.lla1[2] << ","

            << gnss.lla2[0] << ","
            << gnss.lla2[1] << ","
            << gnss.lla2[2] << ","

            << gnss.hAcc1 << ","
            << gnss.vAcc1 << ","

            << gnss.hAcc2 << ","
            << gnss.vAcc2 << ","

            << gnss.position1_enu[0] << ","
            << gnss.position1_enu[1] << ","
            << gnss.position1_enu[2] << ","

            << gnss.position2_enu[0] << ","
            << gnss.position2_enu[1] << ","
            << gnss.position2_enu[2] << ","

            << gnss.position_enu[0] << ","
            << gnss.position_enu[1] << ","
            << gnss.position_enu[2] << ","

            << gnss.baseline_enu[0] << ","
            << gnss.baseline_enu[1] << ","
            << gnss.baseline_enu[2] << ","

            << gnss.baseline_body[0] << ","
            << gnss.baseline_body[1] << ","
            << gnss.baseline_body[2] << ",";

        // ============================================================
        // VEHICLE
        // ============================================================

        report
            << vehicle.velocity << ","
            << vehicle.steering_angle << ","
            << vehicle.wheelbase

            << "\n";
    }

    // ================================================================
    // CLOSE EKF INPUT LOG
    // ================================================================
    void closeReport()
    {
        {
            std::lock_guard<std::mutex> lock(
                report_mutex);

            if (report.is_open())
            {
                report.close();
            }
        }

        // ================================================================
        // STOP GROUND TRUTH ROS BAG
        // ================================================================

        const int bag_result =
            std::system(
                "/ADC_27/scripts/ekf_rosbag.sh stop");

        if (bag_result != 0)
        {
            std::cerr
                << "Failed to stop EKF ground-truth rosbag."
                << std::endl;
        }
    }

}