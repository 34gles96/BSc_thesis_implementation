#include "buildRefHorizont.hpp"
#include "trajectoryConnector.hpp"

#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include <iterator>

bool buildReferenceHorizon(
    const std::vector<TrajectoryPoint> &trajectory,
    double E_current,
    double N_current,
    double yaw_current,
    double v_reference,
    double Ts,
    double refHorizon[40],
    ReferenceHorizonDebug *debug)
{
    constexpr int Np = 10;

    using trajectory_geometry::Point2D;

    // =====================================================
    // 1. PREPARE THE ORIGINAL DESIRED PATH
    // =====================================================

    std::vector<Point2D> original_path;

    for (const auto &point : trajectory)
    {
        if (!std::isfinite(point.E) ||
            !std::isfinite(point.N))
        {
            continue;
        }

        // Remove consecutive duplicate points
        if (!original_path.empty() &&
            std::hypot(
                point.E - original_path.back().E,
                point.N - original_path.back().N) < 1e-4)
        {
            continue;
        }

        original_path.push_back({point.E, point.N});
    }

    if (original_path.size() < 2 ||
        !std::isfinite(E_current) ||
        !std::isfinite(N_current) ||
        !std::isfinite(yaw_current) ||
        !std::isfinite(Ts) || Ts <= 0.0 ||
        !std::isfinite(v_reference) || v_reference < 0.0)
    {
        return false;
    }

    // =====================================================
    // 2. FIND VEHICLE'S CLOSEST PROJECTION
    // =====================================================

    double min_distance_sq =
        std::numeric_limits<double>::max();

    Point2D projection{};

    size_t closest_segment = 0;

    double closest_t = 0;

    for (size_t i = 0; i + 1 < original_path.size(); ++i)
    {
        const double dE =
            original_path[i + 1].E - original_path[i].E;

        const double dN =
            original_path[i + 1].N - original_path[i].N;

        const double len_sq = dE * dE + dN * dN;

        if (len_sq < 1e-12)
            continue;

        // Projection parameter
        double t =
            ((E_current - original_path[i].E) * dE +
             (N_current - original_path[i].N) * dN) /
            len_sq;

        t = std::clamp(t, 0.0, 1.0);

        // Projected coordinates
        const double E_proj =
            original_path[i].E + t * dE;

        const double N_proj =
            original_path[i].N + t * dN;

        const double diff_E = E_current - E_proj;
        const double diff_N = N_current - N_proj;

        const double distance_sq =
            diff_E * diff_E + diff_N * diff_N;

        if (distance_sq < min_distance_sq)
        {
            min_distance_sq = distance_sq;

            closest_segment = i;
            closest_t = t;

            projection = {E_proj, N_proj};
        }
    }

    if (!std::isfinite(min_distance_sq))
        return false;

    const double projection_distance =
        std::sqrt(min_distance_sq);

    const double dE = projection.E - E_current;
    const double dN = projection.N - N_current;

    if (debug != nullptr)
    {
        debug->vehicle_E = E_current;
        debug->vehicle_N = N_current;

        debug->projection_E = projection.E;
        debug->projection_N = projection.N;

        debug->projection_distance = std::hypot(
            E_current - projection.E,
            N_current - projection.N);

        // dot product from pojection
        debug->forward_offset =
            std::cos(yaw_current) * dE +
            std::sin(yaw_current) * dN;

        debug->lateral_offset =
            -std::sin(yaw_current) * dE +
            std::cos(yaw_current) * dN;

        debug->closest_segment = closest_segment;
        debug->closest_segment_t = closest_t;
    }

    // =====================================================
    // 3. CALCULATE DESIRED PATH TANGENT AT PROJECTION
    // =====================================================

    const double yaw_path = std::atan2(
        original_path[closest_segment + 1].N -
            original_path[closest_segment].N,
        original_path[closest_segment + 1].E -
            original_path[closest_segment].E);

    // =====================================================
    // 4. BUILD CUBIC HERMITE CONNECTOR
    // =====================================================

    std::vector<Point2D> connector;

    const Point2D vehicle_position{
        E_current,
        N_current};

    if (!trajectory_geometry::buildCubicHermiteConnector(
            vehicle_position,
            yaw_current,
            projection,
            yaw_path,
            0.2, // Connector sampling spacing [m]
            connector))
    {
        return false;
    }

    if (connector.size() < 2 && projection_distance > 1e-3)
        return false;

    // =====================================================
    // 5. CONSTRUCT THE COMBINED PATH
    //
    // Vehicle -> Hermite connector -> Original trajectory
    // =====================================================

    std::vector<Point2D> combined_path;

    combined_path.reserve(
        connector.size() + original_path.size());

    auto appendUnique = [&combined_path](const Point2D &point)
    {
        if (combined_path.empty() ||
            std::hypot(
                point.E - combined_path.back().E,
                point.N - combined_path.back().N) > 1e-4)
        {
            combined_path.push_back(point);
        }
    };

    // 5/A. Append Hermite connector
    for (const auto &point : connector)
    {
        appendUnique(point);
    }

    // 5/B. Append remaining original trajectory
    // The connector already ends at the projection.
    // Continue from the endpoint of the segment containing it.

    for (size_t i = closest_segment + 1;
         i < original_path.size();
         ++i)
    {
        appendUnique(original_path[i]);
    }

    if (combined_path.size() < 2)
        return false;

    // =====================================================
    // 6. CALCULATE CUMULATIVE DISTANCE ON COMBINED PATH
    // =====================================================

    std::vector<double> s(combined_path.size(), 0.0);

    for (size_t i = 1; i < combined_path.size(); ++i)
    {
        const double dE =
            combined_path[i].E - combined_path[i - 1].E;

        const double dN =
            combined_path[i].N - combined_path[i - 1].N;

        s[i] = s[i - 1] + std::hypot(dE, dN);
    }

    if (debug != nullptr)
    {
        debug->projection_distance = projection_distance;
        debug->connector_points = connector.size();
        debug->combined_path_length = s.back();

        debug->first_reference_advance =
            std::min(v_reference * Ts, s.back());
    }

    // =====================================================
    // 7. GENERATE MPC REFERENCE HORIZON
    // =====================================================

    for (int k = 0; k < Np; ++k)
    {
        // Start from vehicle position (s = 0)
        const double s_ref = std::min(
            (k + 1) * v_reference * Ts,
            s.back());

        // Find segment containing s_ref
        auto it = std::lower_bound(
            s.begin() + 1,
            s.end(),
            s_ref);

        size_t j = static_cast<size_t>(
            std::distance(s.begin(), it));

        if (j >= combined_path.size())
            j = combined_path.size() - 1;

        // Interpolation parameter
        const double segment_length =
            s[j] - s[j - 1];

        if (segment_length < 1e-12)
            return false;

        const double alpha =
            (s_ref - s[j - 1]) / segment_length;

        // Interpolated ENU position
        const double E_ref =
            combined_path[j - 1].E +
            alpha * (combined_path[j].E -
                     combined_path[j - 1].E);

        const double N_ref =
            combined_path[j - 1].N +
            alpha * (combined_path[j].N -
                     combined_path[j - 1].N);

        // Reference heading
        const double yaw_ref = std::atan2(
            combined_path[j].N - combined_path[j - 1].N,
            combined_path[j].E - combined_path[j - 1].E);

        // Stop at the end of available path
        const double v_ref =
            (s_ref >= s.back()) ? 0.0 : v_reference;

        // MATLAB-compatible storage
        refHorizon[4 * k + 0] = E_ref;
        refHorizon[4 * k + 1] = N_ref;
        refHorizon[4 * k + 2] = yaw_ref;
        refHorizon[4 * k + 3] = v_ref;
    }

    return true;
}