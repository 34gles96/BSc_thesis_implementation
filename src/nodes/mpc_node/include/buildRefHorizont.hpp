#pragma once

#include <vector>
#include <cstddef>
#include "mpcInterface.hpp"

struct ReferenceHorizonDebug
{
    double projection_distance = 0.0;
    std::size_t connector_points = 0;
    double combined_path_length = 0.0;
    double first_reference_advance = 0.0;

    double vehicle_E;
    double vehicle_N;

    double projection_E;
    double projection_N;

    double forward_offset = 0.0;
    double lateral_offset = 0.0;

    std::size_t closest_segment = 0;
    double closest_segment_t = 0.0;
};

bool buildReferenceHorizon(
    const std::vector<TrajectoryPoint> &trajectory,
    double E_current,
    double N_current,
    double yaw_current,
    double v_reference,
    double Ts,
    double refHorizon[40],
    ReferenceHorizonDebug *debug = nullptr);