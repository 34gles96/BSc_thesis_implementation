#pragma once

#include "mpcController_terminate.h"
#include "mpcController_initialize.h"
#include "mpcController.h"
#include "mpcController_types.h"

#include <yaml-cpp/yaml.h>
#include "mpcController_types.h"

struct TrajectoryPoint
{
    int32_t u;
    int32_t v;

    double E;
    double N;
    double U;
};

namespace MPC
{
    using mpc_cfg = struct struct0_T;

    // Current state: [E; N; yaw; velocity]
    using EN_y_v = double[4];

    // Outputs
    // double u_optimal[2]{};
    // coder::array<double, 2U> U_optimal;

    inline void init_mpc_cfg(const std::string &path, mpc_cfg &params)
    {
        YAML::Node config = YAML::LoadFile(path);
        YAML::Node mpc = config["mpc"];

        params.Np = mpc["Np"].as<double>();
        params.Ts = mpc["Ts"].as<double>();
        params.L = mpc["L"].as<double>();

        params.deltaMax = mpc["deltaMax"].as<double>();
        params.aMin = mpc["aMin"].as<double>();
        params.aMax = mpc["aMax"].as<double>();

        YAML::Node Q = mpc["Q"];
        YAML::Node R = mpc["R"];
        YAML::Node S = mpc["S"];

        // MATLAB Coder uses column-major storage
        for (size_t row = 0; row < 4; row++)
        {
            for (size_t col = 0; col < 4; col++)
            {
                params.Q[row + 4 * col] = Q[row][col].as<double>();
            }
        }

        for (size_t row = 0; row < 2; row++)
        {
            for (size_t col = 0; col < 2; col++)
            {
                params.R[row + 2 * col] = R[row][col].as<double>();
                params.S[row + 2 * col] = S[row][col].as<double>();
            }
        }
    }
}