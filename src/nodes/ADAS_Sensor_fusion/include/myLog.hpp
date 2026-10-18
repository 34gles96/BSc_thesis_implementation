#pragma once

#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <builtin_interfaces/msg/time.hpp>

#include "EKF_interface.hpp"

namespace myLOG
{

    double stampToMilliseconds(
        const builtin_interfaces::msg::Time &stamp);

    void initializeGnssLog();

    void logGnssLla(
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gnss1,
        const sensor_msgs::msg::NavSatFix::ConstSharedPtr &gnss2);

    void closeGnssLog();

    void initializeEkfLog();

    void logEkfState(
        double timestamp_ms,
        const GeneratedEkfState &state);

    void closeEkfLog();

}