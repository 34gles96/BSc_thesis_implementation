
#include <fstream>
#include <filesystem>
#include <iomanip>
#include <mutex>
#include <iostream>
#include <array>
#include <cstdint>
#include <string>

#include "mpcLogger.hpp"

namespace mpcLoggger
{
    std::ofstream mpc_log;
    std::mutex mpcLog_mutex;
    uint64_t mpc_log_iteration_{0};

    void initializeMPCLog()
    {
        const std::string directory = "/ADC_27/logs/MPC";

        std::filesystem::create_directories(directory);

        std::lock_guard<std::mutex> lock(mpcLog_mutex);

        mpc_log.open(
            directory + "/mpc_replay.csv",
            std::ios::out | std::ios::trunc);

        if (!mpc_log.is_open())
        {
            std::cerr << "Error opening MPC log file" << std::endl;
            return;
        }

        mpc_log_iteration_ = 0;

        mpc_log << std::setprecision(17);

        // Current state and previous control
        mpc_log << "iteration,Ts,"
                << "E_current,N_current,yaw_current,v_current,"
                << "u_last_steering,u_last_acceleration,"
                << "Np,L,deltaMax,aMin,aMax,";

        // Q matrix
        for (int i = 0; i < 16; ++i)
            mpc_log << "Q" << i << ",";

        // R matrix
        for (int i = 0; i < 4; ++i)
            mpc_log << "R" << i << ",";

        // S matrix
        for (int i = 0; i < 4; ++i)
            mpc_log << "S" << i << ",";

        // Reference horizon
        for (int i = 0; i < 40; ++i)
            mpc_log << "ref" << i << ",";

        // First optimal control pair
        mpc_log << "steering_output,"
                << "acceleration_output,";

        // Complete predicted control sequence
        for (int k = 0; k < 10; ++k)
        {
            mpc_log << "U_steering_" << k + 1 << ","
                    << "U_acceleration_" << k + 1 << ",";
        }

        mpc_log << "execution_ms\n";

        std::cout << "MPC log initialized: "
                  << directory << "/mpc_replay.csv"
                  << std::endl;
    }

    void writeMPCLog(
        double Ts,
        const double x_current[4],
        const double refHorizon[40],
        const std::array<double, 2> &u_last,
        const struct0_T &params,
        const double u_optimal[2],
        const coder::array<double, 2U> &U_optimal,
        double execution_ms)
    {
        std::lock_guard<std::mutex> lock(mpcLog_mutex);

        if (!mpc_log.is_open())
            return;

        // Check dimensions of predicted control matrix
        if (U_optimal.size(0) != 2 || U_optimal.size(1) != 10)
        {
            std::cerr << "Invalid U_optimal dimensions: expected 2x10"
                      << std::endl;
            return;
        }

        mpc_log << mpc_log_iteration_++ << ","
                << Ts << ",";

        // Current state [E, N, yaw, velocity]
        for (int i = 0; i < 4; ++i)
            mpc_log << x_current[i] << ",";

        // Previous control input
        for (int i = 0; i < 2; ++i)
            mpc_log << u_last[i] << ",";

        // MPC parameters
        mpc_log << params.Np << ","
                << params.L << ","
                << params.deltaMax << ","
                << params.aMin << ","
                << params.aMax << ",";

        // Q matrix
        for (int i = 0; i < 16; ++i)
            mpc_log << params.Q[i] << ",";

        // R matrix
        for (int i = 0; i < 4; ++i)
            mpc_log << params.R[i] << ",";

        // S matrix
        for (int i = 0; i < 4; ++i)
            mpc_log << params.S[i] << ",";

        // Reference horizon
        for (int i = 0; i < 40; ++i)
            mpc_log << refHorizon[i] << ",";

        // First optimal control pair
        mpc_log << u_optimal[0] << ","
                << u_optimal[1] << ",";

        // Complete predicted control sequence
        // Row 0: steering
        // Row 1: acceleration
        for (int k = 0; k < 10; ++k)
        {
            mpc_log << U_optimal.at(0, k) << ","
                    << U_optimal.at(1, k) << ",";
        }

        // MPC execution duration
        mpc_log << execution_ms << "\n";

        mpc_log.flush();
    }

    void closeMPCLog()
    {
        std::lock_guard<std::mutex> lock(mpcLog_mutex);

        if (mpc_log.is_open())
        {
            mpc_log.flush();
            mpc_log.close();

            std::cout << "MPC log file closed" << std::endl;
        }
    }
}
