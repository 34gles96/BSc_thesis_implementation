#include "readConfig.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

CameraParams loadCameraParams(const std::string &filename)
{
    YAML::Node config = YAML::LoadFile(filename);

    // image_width: 1242
    // image_height: 375

    // Rading (u,v)
    if (!config["image_width"] || !config["image_height"])
    {
        throw std::runtime_error("Invalid YAML format: Missing 'image width' or 'height' node.");
    }

    int image_width = config["image_width"].as<int>();
    int image_height = config["image_height"].as<int>();

    // Reading K1 config matrix of camere
    if (!config["K1"] || !config["K1"]["data"])
    {
        throw std::runtime_error("Invalid YAML format: Missing 'K1' or 'data' node.");
    }

    // Read the 3x3 K matrix array [fx, 0, cx, 0, fy, cy, 0, 0, 1]
    std::vector<double> k_data = config["K1"]["data"].as<std::vector<double>>();

    if (k_data.size() < 9)
    {
        throw std::runtime_error("K matrix data must contain at least 9 elements.");
    }

    // reading baslink vec -> baslink distance
    std::vector<double> baselink_vec = config["T"]["data"].as<std::vector<double>>();

    if (baselink_vec.size() != 3)
    {
        throw std::runtime_error("T matrix data must contain 3 elements");
    }

    double baselink_d = sqrt(baselink_vec[0] * baselink_vec[0] +
                             baselink_vec[1] * baselink_vec[1] +
                             baselink_vec[2] * baselink_vec[2]);

    CameraParams params;

    // Extract intrinsic parameters from standard K matrix layout:
    // [0] = fx, [2] = cx
    // [4] = fy, [5] = cy
    params.fx = k_data[0];
    params.cx = k_data[2];
    params.fy = k_data[4];
    params.cy = k_data[5];
    params.baselink = baselink_d;
    params.width = image_width;
    params.height = image_height;

    return params;
}
