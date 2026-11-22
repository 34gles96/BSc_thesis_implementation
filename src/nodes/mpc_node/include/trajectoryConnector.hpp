
#pragma once

#include <vector>

namespace trajectory_geometry
{

    struct Point2D
    {
        double E;
        double N;
    };

    bool buildCubicHermiteConnector(
        const Point2D &start,
        double start_yaw,
        const Point2D &end,
        double end_yaw,
        double sample_spacing,
        std::vector<Point2D> &connector);

}
