#include "unicycle_ugv_controller/nmpc/unicycle_nmpc_solver.h"

#include <ros/console.h>

#include <chrono>

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
    resetWarmStart();
    return true;
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
        }
        for (int i = 0; i < UNICYCLE_NMPC_N; ++i) {
            u_guess_[static_cast<size_t>(i)] =
                control::packControl(refs[static_cast<size_t>(i)].control);
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
