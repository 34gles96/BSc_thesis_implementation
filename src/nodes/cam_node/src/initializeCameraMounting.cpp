
#include <opencv2/core.hpp>
#include <stdexcept>
#include <string>
#include <iostream>
#include <array>
#include <algorithm>
#include <cmath>
#include <yaml-cpp/yaml.h>

#include "initializeCameraMounting.hpp"

CamType::Transform4x4_Cam2Body initializeCameraMounting(
    const std::string &yamlPath)
{
    YAML::Node cfg;

    try
    {
        cfg = YAML::LoadFile(yamlPath);
    }
    catch (const YAML::Exception &e)
    {
        std::cerr << "Error opening camera YAML: "
                  << e.what() << std::endl;
        throw;
    }

    // Camera mounting position expressed in BeamNG frame
    CamType::CamPos_3d_V cam_pos_V{};

    YAML::Node cam_pos_node =
        cfg["camera_L_pos_V"]["data"];

    for (std::size_t i = 0;
         i < cfg["camera_L_pos_V"]["rows"].as<int>(); i++)
    {
        cam_pos_V[i] = cam_pos_node[i].as<double>();
    }

    // Camera forward direction expressed in BeamNG frame
    CamType::camera_dir_3d_V cam_dir_V{};

    YAML::Node cam_dir_node =
        cfg["camera_L_dir_V"]["data"];

    for (std::size_t i = 0;
         i < cfg["camera_L_dir_V"]["rows"].as<int>(); i++)
    {
        cam_dir_V[i] = cam_dir_node[i].as<double>();
    }

    // Camera upward direction expressed in BeamNG frame
    CamType::camera_up_3d_V cam_up_V{};

    YAML::Node cam_up_node =
        cfg["camera_L_up_V"]["data"];

    for (std::size_t i = 0;
         i < cfg["camera_L_up_V"]["rows"].as<int>(); i++)
    {
        cam_up_V[i] = cam_up_node[i].as<double>();
    }

    // Validate numerical values
    auto isValidVector = [](const std::array<double, 3> &m) -> bool
    {
        return std::all_of(m.begin(), m.end(),
                           [](double value)
                           {
                               return std::isfinite(value);
                           });
    };

    auto isValidDirection = [&](const std::array<double, 3> &m) -> bool
    {
        if (!isValidVector(m))
            return false;

        const double normSquared =
            m[0] * m[0] +
            m[1] * m[1] +
            m[2] * m[2];

        return normSquared > 1e-12;
    };

    if (!isValidDirection(cam_up_V) ||
        !isValidDirection(cam_dir_V))
    {
        throw std::runtime_error(
            "Invalid camera orientation vectors");
    }

    // Assumed CoG origin expressed in BeamNG frame
    std::array<double, 3> cog_v{};

    // Homogeneous transformation matrix
    CamType::Transform4x4_Cam2Body T_cam2body{};

    initializeCameraToCoGTransform(
        cam_pos_V.data(),
        cam_dir_V.data(),
        cam_up_V.data(),
        cog_v.data(),
        T_cam2body.data());

    return T_cam2body;
}
