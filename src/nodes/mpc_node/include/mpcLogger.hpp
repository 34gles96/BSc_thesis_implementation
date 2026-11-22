
#pragma once

#include <array>
#include "mpcInterface.hpp"

namespace mpcLoggger
{
    void initializeMPCLog();

    void writeMPCLog(
        double Ts,
        const double x_current[4],
        const double refHorizon[40],
        const std::array<double, 2> &u_last,
        const struct0_T &params,
        const double u_optimal[2],
        const coder::array<double, 2U> &U_optimal,
        double execution_ms);

    void closeMPCLog();
}
