#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace unicycle_ugv_controller {

// Optional centre of the existing damped inverse. Zero gain preserves the
// production law. No angle/speed mode switches or additional integrator.
struct HeadingRecoveryConfig {
    double gain{0.0};       // s^-1; opt in only after platform validation
    double axis_bias{0.5};  // (0,1): prefer forward, retain a reverse minimum
    double rate_damping{0.5};

    void validate() const {
        if (!std::isfinite(gain) || gain < 0.0 || !std::isfinite(axis_bias) || axis_bias <= 0.0 ||
            axis_bias >= 1.0 || !std::isfinite(rate_damping) || rate_damping < 0.0) {
            throw std::invalid_argument("invalid flatness heading_recovery parameters");
        }
    }
};

struct HeadingRecoveryResult {
    double centre{0.0};
    double potential{0.0};
    double descent{0.0};
    bool valid{false};
};

// h is a desired world velocity, e.g. v_ref + (kp/kv)*(p_ref-p).
// d=h/sqrt(|h|^2+eps^2), r=|d|, c=t(theta)'d, e=n(theta)'d.
// U=0.5*e^2 + bias*r*(r-c), -dU/dtheta=e*(c+bias*r).
// Both forward and reverse axes are local minima (0<bias<1); at 90 degrees
// the restoring term is nonzero. At h=0 it vanishes without atan2(0,0).
// For fixed h and ideal yaw actuation, omega=gain*(-dU/dtheta) dissipates U.
// This limited identity is NOT a full moving-reference/skid-steer proof.
inline HeadingRecoveryResult headingRecoveryCentre(double hx, double hy, double yaw,
                                                   double yaw_rate, double reference_rate,
                                                   double eps, const HeadingRecoveryConfig& cfg) {
    cfg.validate();
    HeadingRecoveryResult out;
    if (!std::isfinite(hx) || !std::isfinite(hy) || !std::isfinite(yaw) ||
        !std::isfinite(yaw_rate) || !std::isfinite(reference_rate) || !std::isfinite(eps) ||
        eps <= 0.0)
        return out;
    if (cfg.gain == 0.0) {
        out.valid = true;
        return out;
    }
    // Scaled normalisation avoids squared-norm overflow for finite inputs.
    const double scale = std::max({std::abs(hx), std::abs(hy), eps});
    const double x = hx / scale, y = hy / scale, z = eps / scale;
    const double den = std::sqrt(x * x + y * y + z * z);
    const double dx = x / den, dy = y / den;
    const double r = std::hypot(dx, dy);
    const double c = std::cos(yaw) * dx + std::sin(yaw) * dy;
    const double e = -std::sin(yaw) * dx + std::cos(yaw) * dy;
    out.descent = e * (c + cfg.axis_bias * r);
    out.potential = 0.5 * e * e + cfg.axis_bias * r * (r - c);
    out.centre =
        reference_rate + cfg.gain * out.descent - cfg.rate_damping * (yaw_rate - reference_rate);
    out.valid = std::isfinite(out.centre) && std::isfinite(out.potential);
    return out;
}

// Unique minimiser of (speed*w-a_perp)^2 + eps^2*(w-centre)^2.
// Compute both coefficients without forming speed^2 or dividing by speed.
inline bool centredDampedYawRate(double speed, double a_perp, double eps, double centre,
                                 double& result) {
    result = 0.0;
    if (!std::isfinite(speed) || !std::isfinite(a_perp) || !std::isfinite(eps) || eps <= 0.0 ||
        !std::isfinite(centre))
        return false;
    const double scale = std::max(std::abs(speed), eps);
    const double v = speed / scale, e = eps / scale;
    const double den = v * v + e * e;
    result = ((v / den) / scale) * a_perp + (e * e / den) * centre;
    return std::isfinite(result);
}

// Bounded course-rate surrogate for feedforward, never a zero-speed inverse.
inline bool regularizedReferenceRate(double vx, double vy, double ax, double ay, double eps,
                                     double& rate) {
    rate = 0.0;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(ax) || !std::isfinite(ay) ||
        !std::isfinite(eps) || eps <= 0.0)
        return false;
    const double scale = std::max({std::abs(vx), std::abs(vy), eps});
    const double x = vx / scale, y = vy / scale, e = eps / scale;
    const double den = x * x + y * y + e * e;
    rate = ((x / den) / scale) * ay - ((y / den) / scale) * ax;
    return std::isfinite(rate);
}

}  // namespace unicycle_ugv_controller
