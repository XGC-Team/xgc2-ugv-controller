#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"

#include <ros/console.h>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace unicycle_ugv_controller {

namespace {

bool finiteVector(const Eigen::VectorXd& value) {
    return value.array().isFinite().all();
}

}  // namespace

UnicycleNmpcSolver::UnicycleNmpcSolver() = default;

UnicycleNmpcSolver::~UnicycleNmpcSolver() {
    cleanup();
}

bool UnicycleNmpcSolver::initialize() {
    if (initialized_) {
        return true;
    }
    if (UNICYCLE_NMPC_NX != 4 || UNICYCLE_NMPC_NU != 2 || UNICYCLE_NMPC_NP != 6 ||
        UNICYCLE_NMPC_N != 10) {
        ROS_ERROR("[UnicycleNmpcSolver] Unexpected generated solver dimensions");
        return false;
    }
    capsule_ = unicycle_nmpc_acados_create_capsule();
    if (!capsule_) {
        ROS_ERROR("[UnicycleNmpcSolver] Failed to create acados capsule");
        return false;
    }
    const int status = unicycle_nmpc_acados_create(capsule_);
    if (status != 0) {
        ROS_ERROR("[UnicycleNmpcSolver] unicycle_nmpc_acados_create failed: %d", status);
        cleanup();
        return false;
    }
    initialized_ = true;
    if (!applyRuntimeBounds()) {
        cleanup();
        return false;
    }
    resetWarmStart();
    return true;
}

bool UnicycleNmpcSolver::configureBounds(double min_linear_speed, double max_linear_speed,
                                         double max_linear_acceleration,
                                         double max_angular_speed) {
    if (!std::isfinite(min_linear_speed) || !std::isfinite(max_linear_speed) ||
        min_linear_speed >= max_linear_speed || !std::isfinite(max_linear_acceleration) ||
        max_linear_acceleration <= 0.0 || !std::isfinite(max_angular_speed) ||
        max_angular_speed <= 0.0) {
        ROS_ERROR(
            "[UnicycleNmpcSolver] Invalid runtime bounds speed=[%.3f, %.3f] accel=%.3f "
            "omega=%.3f",
            min_linear_speed, max_linear_speed, max_linear_acceleration, max_angular_speed);
        return false;
    }

    const std::array<double, UNICYCLE_NMPC_NU> input_lower{
        {-max_linear_acceleration, -max_angular_speed}};
    const std::array<double, UNICYCLE_NMPC_NU> input_upper{
        {max_linear_acceleration, max_angular_speed}};
    const bool changed = input_lower != input_lower_bounds_ || input_upper != input_upper_bounds_ ||
                         speed_lower_bound_[0] != min_linear_speed ||
                         speed_upper_bound_[0] != max_linear_speed;
    input_lower_bounds_ = input_lower;
    input_upper_bounds_ = input_upper;
    speed_lower_bound_[0] = min_linear_speed;
    speed_upper_bound_[0] = max_linear_speed;
    if (!changed || !capsule_) {
        return true;
    }
    resetWarmStart();
    return applyRuntimeBounds();
}

void UnicycleNmpcSolver::resetWarmStart() {
    have_warm_start_ = false;
    for (auto& x : x_guess_) {
        x.setZero();
    }
    for (auto& u : u_guess_) {
        u.setZero();
    }
}

bool UnicycleNmpcSolver::solve(const Se2StateVector& x0, const std::vector<Se2Reference>& refs) {
    if (!initialized_ && !initialize()) {
        return false;
    }
    if (refs.size() != static_cast<size_t>(UNICYCLE_NMPC_N + 1) || !finiteVector(x0)) {
        return false;
    }
    if (!setInitialState(x0)) {
        return false;
    }
    for (int i = 0; i <= UNICYCLE_NMPC_N; ++i) {
        if (!setReference(i, refs[static_cast<size_t>(i)])) {
            return false;
        }
    }
    setGuesses(x0, refs);
    const auto t0 = std::chrono::steady_clock::now();
    solver_status_ = unicycle_nmpc_acados_solve(capsule_);
    const auto t1 = std::chrono::steady_clock::now();
    solve_time_ms_ = std::chrono::duration<double, std::milli>(t1 - t0).count();
    if (solver_status_ != 0) {
        ROS_WARN_THROTTLE(1.0, "[UnicycleNmpcSolver] Solve failed with status %d", solver_status_);
        return false;
    }
    readSolution();
    shiftWarmStart();
    return true;
}

bool UnicycleNmpcSolver::setInitialState(const Se2StateVector& x0) {
    ocp_nlp_config* config = unicycle_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = unicycle_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = unicycle_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = unicycle_nmpc_acados_get_nlp_out(capsule_);
    int status = ocp_nlp_constraints_model_set(config, dims, in, out, 0, "lbx",
                                               const_cast<double*>(x0.data()));
    status |= ocp_nlp_constraints_model_set(config, dims, in, out, 0, "ubx",
                                            const_cast<double*>(x0.data()));
    return status == 0;
}

bool UnicycleNmpcSolver::applyRuntimeBounds() {
    if (!capsule_) {
        return false;
    }
    ocp_nlp_config* config = unicycle_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = unicycle_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = unicycle_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = unicycle_nmpc_acados_get_nlp_out(capsule_);

    int status = 0;
    for (int i = 0; i < UNICYCLE_NMPC_N; ++i) {
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "lbu",
                                                input_lower_bounds_.data());
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "ubu",
                                                input_upper_bounds_.data());
    }
    for (int i = 1; i <= UNICYCLE_NMPC_N; ++i) {
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "lbx",
                                                speed_lower_bound_.data());
        status |= ocp_nlp_constraints_model_set(config, dims, in, out, i, "ubx",
                                                speed_upper_bound_.data());
    }
    if (status != 0) {
        ROS_ERROR("[UnicycleNmpcSolver] Failed to apply runtime input/state bounds");
        return false;
    }
    ROS_INFO("[UnicycleNmpcSolver] Runtime bounds speed=[%.3f, %.3f] accel=%.3f omega=%.3f",
             speed_lower_bound_[0], speed_upper_bound_[0], input_upper_bounds_[0],
             input_upper_bounds_[1]);
    return true;
}

bool UnicycleNmpcSolver::setReference(int stage, const Se2Reference& ref) {
    Eigen::Matrix<double, UNICYCLE_NMPC_NP, 1> p;
    p.segment<4>(0) = control::packState(ref.state);
    p.segment<2>(4) = control::packControl(ref.control);
    return unicycle_nmpc_acados_update_params(capsule_, stage, p.data(), UNICYCLE_NMPC_NP) == 0;
}

void UnicycleNmpcSolver::setGuesses(const Se2StateVector& x0,
                                    const std::vector<Se2Reference>& refs) {
    ocp_nlp_config* config = unicycle_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = unicycle_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_in* in = unicycle_nmpc_acados_get_nlp_in(capsule_);
    ocp_nlp_out* out = unicycle_nmpc_acados_get_nlp_out(capsule_);
    if (!have_warm_start_) {
        for (int i = 0; i <= UNICYCLE_NMPC_N; ++i) {
            x_guess_[static_cast<size_t>(i)] =
                control::packState(refs[static_cast<size_t>(i)].state);
            x_guess_[static_cast<size_t>(i)](3) =
                std::clamp(x_guess_[static_cast<size_t>(i)](3), speed_lower_bound_[0],
                           speed_upper_bound_[0]);
        }
        for (int i = 0; i < UNICYCLE_NMPC_N; ++i) {
            u_guess_[static_cast<size_t>(i)] =
                control::packControl(refs[static_cast<size_t>(i)].control);
            for (int j = 0; j < UNICYCLE_NMPC_NU; ++j) {
                u_guess_[static_cast<size_t>(i)](j) =
                    std::clamp(u_guess_[static_cast<size_t>(i)](j),
                               input_lower_bounds_[static_cast<size_t>(j)],
                               input_upper_bounds_[static_cast<size_t>(j)]);
            }
        }
    }
    x_guess_[0] = x0;
    for (int i = 0; i <= UNICYCLE_NMPC_N; ++i) {
        ocp_nlp_out_set(config, dims, out, in, i, "x", x_guess_[static_cast<size_t>(i)].data());
        if (i < UNICYCLE_NMPC_N) {
            ocp_nlp_out_set(config, dims, out, in, i, "u", u_guess_[static_cast<size_t>(i)].data());
        }
    }
}

void UnicycleNmpcSolver::readSolution() {
    ocp_nlp_config* config = unicycle_nmpc_acados_get_nlp_config(capsule_);
    ocp_nlp_dims* dims = unicycle_nmpc_acados_get_nlp_dims(capsule_);
    ocp_nlp_out* out = unicycle_nmpc_acados_get_nlp_out(capsule_);
    for (int i = 0; i <= UNICYCLE_NMPC_N; ++i) {
        ocp_nlp_out_get(config, dims, out, i, "x", x_solution_[static_cast<size_t>(i)].data());
        if (i < UNICYCLE_NMPC_N) {
            ocp_nlp_out_get(config, dims, out, i, "u", u_solution_[static_cast<size_t>(i)].data());
        }
    }
    optimal_control_ = u_solution_[0];
    predicted_speed_ = x_solution_[1](3);
}

void UnicycleNmpcSolver::shiftWarmStart() {
    for (int i = 0; i <= UNICYCLE_NMPC_N; ++i) {
        x_guess_[static_cast<size_t>(i)] = x_solution_[static_cast<size_t>(i)];
    }
    for (int i = 0; i < UNICYCLE_NMPC_N; ++i) {
        u_guess_[static_cast<size_t>(i)] = u_solution_[static_cast<size_t>(i)];
    }
    have_warm_start_ = true;
}

void UnicycleNmpcSolver::cleanup() {
    if (capsule_) {
        if (initialized_) {
            unicycle_nmpc_acados_free(capsule_);
        }
        unicycle_nmpc_acados_free_capsule(capsule_);
        capsule_ = nullptr;
    }
    initialized_ = false;
}

}  // namespace unicycle_ugv_controller
