#pragma once
#include <cmath>

namespace dt46_vision {
struct BallisticSolution {
    double pitch = 0;
    double flight_time = 0;
    bool valid = false;
};

// Low arc, gravity + isotropic quadratic drag. No table or inverse iteration.
inline BallisticSolution solve_projectile(double range, double height, double speed, double drag) {
    if (!std::isfinite(range) || !std::isfinite(height) || !std::isfinite(speed) || !std::isfinite(drag) ||
        range < .2 || range > 12 || std::abs(height) > 3.5 || speed <= 0 || drag < 0) return {};
    const double a0 = 9.81 * range * range / (2 * speed * speed);
    const double vacuum_discriminant = range * range - 4 * a0 * (a0 + height);
    if (vacuum_discriminant <= 0) return {};
    const double vacuum_slope = 2 * (a0 + height) / (range + std::sqrt(vacuum_discriminant));
    // First-order drag contribution to drop weights path position by (range-x)^2;
    // its centroid is range/4. Evaluate the known vacuum slope there, once.
    const double slope_at_quarter = vacuum_slope - a0 * (1 + vacuum_slope * vacuum_slope) / (2 * range);
    const double q = drag * std::hypot(1., slope_at_quarter) * range;
    const double drop_factor = q < .001 ? 1 + q * (2./3 + q * (1./3 + q * 2./15)) :
                               (std::expm1(2*q) - 2*q) / (2*q*q);
    const double a = a0 * drop_factor;
    const double discriminant = range * range - 4 * a * (a + height);
    if (!std::isfinite(discriminant) || discriminant <= 0) return {};
    const double slope = 2 * (a + height) / (range + std::sqrt(discriminant));
    const double time_factor = q == 0 ? 1 : std::expm1(q) / q;
    const double time = range / speed * std::hypot(1., slope) * time_factor;
    if (!std::isfinite(time) || time <= 0) return {};
    return {std::atan(slope), time, true};
}
}
