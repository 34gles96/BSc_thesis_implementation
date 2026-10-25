#include <algorithm>
#include <cmath>

double steeringToBeamNG(double delta) // from experimental resuls
{
    constexpr double a = -0.383825;
    constexpr double b = -0.003022;

    // Inverse of delta = a*u + b
    const double u = (delta - b) / a;

    // BeamNG accepts normalized steering in [-1, 1]
    return std::clamp(u, -1.0, 1.0);
}