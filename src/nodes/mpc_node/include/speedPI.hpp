
#pragma once

#include <algorithm>
#include <cmath>

struct PedalCommand
{
    double throttle{0.0};
    double brake{0.0};
};

class SpeedPI
{
public:
    SpeedPI(double Kp, double Ki)
        : Kp_(Kp), Ki_(Ki)
    {
    }

    PedalCommand update(
        double v_target,
        double v_current,
        double Ts)
    {
        if (!std::isfinite(v_target) ||
            !std::isfinite(v_current) ||
            !std::isfinite(Ts) || Ts <= 0.0)
        {
            return {};
        }

        const double error = v_target - v_current;

        // PI output using the stored integral
        const double u_unsaturated =
            Kp_ * error + Ki_ * integral_;

        // Conditional integration anti-windup
        if ((u_unsaturated > -1.0 && u_unsaturated < 1.0) ||
            (u_unsaturated >= 1.0 && error < 0.0) ||
            (u_unsaturated <= -1.0 && error > 0.0))
        {
            integral_ += error * Ts;
        }

        // Compute PI output using updated integral
        const double u_PI = std::clamp(
            Kp_ * error + Ki_ * integral_,
            -1.0,
            1.0);

        PedalCommand command;

        command.throttle = std::max(0.0, u_PI);
        command.brake = std::max(0.0, -u_PI);

        return command;
    }

    void reset()
    {
        integral_ = 0.0;
    }

private:
    double Kp_;
    double Ki_;
    double integral_{0.0};
};
