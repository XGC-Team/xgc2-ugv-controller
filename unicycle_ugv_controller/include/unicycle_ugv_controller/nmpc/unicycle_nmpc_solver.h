#pragma once

#include <Eigen/Dense>
#include <array>
#include <vector>
#include <xgc2_math/control.hpp>

extern "C" {
#include "acados_c/ocp_nlp_interface.h"
#include "acados_solver_unicycle_nmpc.h"
}

namespace unicycle_ugv_controller {

namespace control = xgc2_math::control;
using Se2ControlVector = control::Se2ControlVector;
using Se2Reference = control::Se2Reference;
using Se2StateVector = control::Se2StateVector;

class UnicycleNmpcSolver {
   public:
    UnicycleNmpcSolver();
    ~UnicycleNmpcSolver();

    UnicycleNmpcSolver(const UnicycleNmpcSolver&) = delete;
    UnicycleNmpcSolver& operator=(const UnicycleNmpcSolver&) = delete;

    bool initialize();
    bool configureBounds(double min_linear_speed, double max_linear_speed,
                         double max_linear_acceleration, double max_angular_speed);
    void resetWarmStart();
    bool solve(const Se2StateVector& x0, const std::vector<Se2Reference>& refs);

    Se2ControlVector optimalControl() const {
        return optimal_control_;
    }
    double predictedSpeed() const {
        return predicted_speed_;
    }
    int status() const {
        return solver_status_;
    }
    double solveTimeMs() const {
        return solve_time_ms_;
    }
    const std::array<Se2StateVector, UNICYCLE_NMPC_N + 1>& predictedStates() const {
        return x_solution_;
    }
    size_t predictedStateCount() const {
        return static_cast<size_t>(UNICYCLE_NMPC_N + 1);
    }
    static constexpr int horizonSteps() {
        return UNICYCLE_NMPC_N;
    }

   private:
    bool applyRuntimeBounds();
    bool setInitialState(const Se2StateVector& x0);
    bool setReference(int stage, const Se2Reference& ref);
    void setGuesses(const Se2StateVector& x0, const std::vector<Se2Reference>& refs);
    void readSolution();
    void shiftWarmStart();
    void cleanup();

    unicycle_nmpc_solver_capsule* capsule_{nullptr};
    bool initialized_{false};
    bool have_warm_start_{false};
    int solver_status_{-1};
    double solve_time_ms_{0.0};
    std::array<Se2StateVector, UNICYCLE_NMPC_N + 1> x_guess_{};
    std::array<Se2ControlVector, UNICYCLE_NMPC_N> u_guess_{};
    std::array<Se2StateVector, UNICYCLE_NMPC_N + 1> x_solution_{};
    std::array<Se2ControlVector, UNICYCLE_NMPC_N> u_solution_{};
    std::array<double, UNICYCLE_NMPC_NU> input_lower_bounds_{{-2.0, -2.5}};
    std::array<double, UNICYCLE_NMPC_NU> input_upper_bounds_{{2.0, 2.5}};
    std::array<double, 1> speed_lower_bound_{{-1.5}};
    std::array<double, 1> speed_upper_bound_{{3.0}};
    Se2ControlVector optimal_control_{Se2ControlVector::Zero()};
    double predicted_speed_{0.0};
};

}  // namespace unicycle_ugv_controller
