#pragma once

#include <yaml-cpp/yaml.h>
#include <string>

#include "EKF_interface.hpp"

struct VehicleParams
{
    double wheelbase;
};

inline GeneratedImuMounting loadImuMounting(
    const std::string &configPath)
{
    GeneratedImuMounting mounting{};

    YAML::Node config =
        YAML::LoadFile(configPath);

    YAML::Node imu =
        config["imu"]["mounting"];

    for (int i = 0; i < 3; ++i)
    {
        mounting.x_sensor_body[i] =
            imu["x_sensor_body"][i].as<double>();

        mounting.y_sensor_body[i] =
            imu["y_sensor_body"][i].as<double>();

        mounting.z_sensor_body[i] =
            imu["z_sensor_body"][i].as<double>();
    }

    return mounting;
}

inline VehicleParams loadVehicleParams(
    const std::string &configPath)
{
    YAML::Node config =
        YAML::LoadFile(configPath);

    VehicleParams params{};

    params.wheelbase =
        config["vehicle"]["wheelbase"].as<double>();

    return params;
}