
#include "trajectoryConnector.hpp"

#include <algorithm>
#include <cmath>

namespace trajectory_geometry
{

    bool buildCubicHermiteConnector(
        const Point2D &start,
        double start_yaw,
        const Point2D &end,
        double end_yaw,
        double sample_spacing,
        std::vector<Point2D> &connector)
    {
        connector.clear();

        // ============================================
        // 1. Validate input
        // ============================================

        if (!std::isfinite(start.E) ||
            !std::isfinite(start.N) ||
            !std::isfinite(end.E) ||
            !std::isfinite(end.N) ||
            !std::isfinite(start_yaw) ||
            !std::isfinite(end_yaw) ||
            !std::isfinite(sample_spacing) ||
            sample_spacing <= 0.0)
        {
            return false;
        }

        // ============================================
        // 2. Calculate endpoint distance
        // ============================================

        const double dE = end.E - start.E;
        const double dN = end.N - start.N;

        const double distance = std::hypot(dE, dN);

        connector.push_back(start);

        if (distance < 1e-6)
        {
            connector.back() = end;
            return true;
        }

        // ============================================
        // 3. Calculate tangent vectors
        // ============================================

        // Initial tangent magnitude heuristic
        const double tangent_length =
            std::min(0.5 * distance, 3.0);

        // Initial tangent: vehicle heading
        const double M0_E =
            tangent_length * std::cos(start_yaw);

        const double M0_N =
            tangent_length * std::sin(start_yaw);

        // Final tangent: desired trajectory heading
        const double M1_E =
            tangent_length * std::cos(end_yaw);

        const double M1_N =
            tangent_length * std::sin(end_yaw);

        // ============================================
        // 4. Determine number of samples
        // ============================================

        const double estimated_length =
            distance + 2.0 * tangent_length;

        const int segments = static_cast<int>(
            std::max(
                2.0,
                std::min(
                    1000.0,
                    std::ceil(estimated_length / sample_spacing))));

        // ============================================
        // 5. Generate cubic Hermite curve
        // ============================================

        for (int i = 1; i <= segments; ++i)
        {
            // Parameter t in [0, 1]
            const double t =
                static_cast<double>(i) / segments;

            const double t2 = t * t;
            const double t3 = t2 * t;

            // Hermite basis functions
            const double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
            const double h10 = t3 - 2.0 * t2 + t;
            const double h01 = -2.0 * t3 + 3.0 * t2;
            const double h11 = t3 - t2;

            Point2D point;

            // East coordinate
            point.E =
                h00 * start.E +
                h10 * M0_E +
                h01 * end.E +
                h11 * M1_E;

            // North coordinate
            point.N =
                h00 * start.N +
                h10 * M0_N +
                h01 * end.N +
                h11 * M1_N;

            // Guarantee exact endpoint
            if (i == segments)
            {
                point = end;
            }

            connector.push_back(point);
        }

        return true;
    }

}
