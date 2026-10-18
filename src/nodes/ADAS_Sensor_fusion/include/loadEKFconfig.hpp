#pragma once

#include <array>
#include <string>
#include <yaml-cpp/yaml.h>

struct EkfConfig
{
    std::array<double, 3> gyro_bias_std;
    std::array<double, 2> bicycle_position_std;
};

inline EkfConfig loadEkfConfig(
    const std::string &configPath)
{
    YAML::Node config =
        YAML::LoadFile(configPath);

    YAML::Node ekf = config["ekf"];

    EkfConfig params{};

    for (int i = 0; i < 3; ++i)
    {
        params.gyro_bias_std[i] =
            ekf["initial"]
               ["gyro_bias_std"][i]
                   .as<double>();
    }

    for (int i = 0; i < 2; ++i)
    {
        params.bicycle_position_std[i] =
            ekf["process_noise"]
               ["bicycle_position_std"][i]
                   .as<double>();
    }

    return params;
}