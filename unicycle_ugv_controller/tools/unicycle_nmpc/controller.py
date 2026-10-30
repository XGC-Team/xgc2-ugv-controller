from __future__ import annotations

from dataclasses import dataclass, field
import os
from pathlib import Path
from typing import List, Optional

import numpy as np


@dataclass(frozen=True)
class Bounds:
    a_min: float = -2.0
    a_max: float = 2.0
    alpha_min: float = -3.0
    alpha_max: float = 3.0
    omega_min: float = -2.5
    omega_max: float = 2.5
    v_min: float = -1.5
    v_max: float = 3.0


@dataclass(frozen=True)
class CostWeights:
    position: np.ndarray = field(default_factory=lambda: np.array([20.0, 20.0]))
    yaw: float = 8.0
    speed: float = 4.0
    omega: float = 10.0
    control: np.ndarray = field(default_factory=lambda: np.array([0.4, 1.0]))
    terminal_position: np.ndarray = field(default_factory=lambda: np.array([60.0, 60.0]))
    terminal_yaw: float = 20.0
    terminal_speed: float = 10.0


@dataclass(frozen=True)
class MPCConfig:
    horizon: float = 1.0
    steps: int = 10
    max_iter: int = 20
    ftol: float = 1.0e-4
    model_name: str = "unicycle_nmpc"
    code_export_directory: Optional[str] = None
    json_file: Optional[str] = None

    @property
    def dt(self) -> float:
        if self.steps <= 0:
            raise ValueError("steps must be positive")
        if self.horizon <= 0.0:
            raise ValueError("horizon must be positive")
        return self.horizon / self.steps


# The single installed SDK. Discovery, alternate roots, sys.path/library-path
# rewriting and ctypes preloading are intentionally absent: the released
# package setup selects the code-generation interpreter and this module only
# validates the canonical installation read-only.
ACADOS_ROOT = Path("/opt/xgc2/acados")
ACADOS_REF = "v0.6.0"
ACADOS_SHA = "503364817c872d474ab5bed219c26760ac267769"
ACADOS_PACKAGE_BASE = "0.6.0-1"


def _installed_acados_status() -> dict[str, bool | str]:
    """Validate the installed xgc2-acados SDK without mutating the process."""

    root = ACADOS_ROOT
    status: dict[str, bool | str] = {
        "acados_root": str(root),
        "acados_root_exists": root.is_dir(),
        "casadi": False,
        "acados_template": False,
    }
    conflict = os.environ.get("ACADOS_SOURCE_DIR")
    if conflict:
        try:
            resolved = Path(conflict).expanduser().resolve()
        except OSError:
            resolved = None
        if resolved != root.resolve():
            status["error"] = (
                f"ACADOS_SOURCE_DIR={conflict} conflicts with the installed SDK at {root}"
            )
            return status
    if not root.is_dir():
        status["error"] = f"xgc2-acados is not installed at {root}; install the released package"
        return status
    try:
        lock = dict(
            line.split("=", 1)
            for line in (root / "acados.lock").read_text().splitlines()
            if "=" in line
        )
    except OSError as error:
        status["error"] = f"cannot read the installed SDK lock {root / 'acados.lock'}: {error}"
        return status
    lock = {key.strip(): value.strip() for key, value in lock.items()}
    if (
        lock.get("ACADOS_REF") != ACADOS_REF
        or lock.get("ACADOS_SHA") != ACADOS_SHA
        or lock.get("XGC2_PACKAGE_VERSION") != ACADOS_PACKAGE_BASE
    ):
        status["error"] = (
            f"installed SDK lock is not the selected acados {ACADOS_REF} "
            f"({ACADOS_SHA}, package base {ACADOS_PACKAGE_BASE})"
        )
        return status
    try:
        import acados_template
        import casadi
    except Exception as error:
        status["error"] = (
            f"casadi/acados_template is not importable from the SDK environment: {error}"
        )
        return status
    template_path = Path(acados_template.__file__).resolve()
    casadi_path = Path(casadi.__file__).resolve()
    if root.resolve() not in template_path.parents:
        status["error"] = (
            f"acados_template is loaded from {template_path}, outside the installed SDK {root}"
        )
        return status
    if (root / "python").resolve() not in casadi_path.parents:
        status["error"] = (
            f"casadi is loaded from {casadi_path}, outside the installed SDK {root}/python"
        )
        return status
    status["casadi"] = True
    status["acados_template"] = True
    return status


class AcadosBackendUnavailable(RuntimeError):
    pass


class AcadosUnicycleNMPC:
    def __init__(
        self,
        config: Optional[MPCConfig] = None,
        bounds: Optional[Bounds] = None,
        weights: Optional[CostWeights] = None,
    ) -> None:
        status = _installed_acados_status()
        if not status["casadi"] or not status["acados_template"]:
            reason = status.get("error", "the installed xgc2-acados SDK is not available")
            raise AcadosBackendUnavailable(
                f"acados backend requires the installed xgc2-acados SDK: {reason}"
            )
        self.config = config if config is not None else MPCConfig()
        self.bounds = bounds if bounds is not None else Bounds()
        self.weights = weights if weights is not None else CostWeights()

    def _build_ocp(self):
        import casadi as ca
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        nx = 5
        nu = 2
        np_param = nx + nu
        x = ca.SX.sym("x", nx)
        xdot = ca.SX.sym("xdot", nx)
        u = ca.SX.sym("u", nu)
        p = ca.SX.sym("p", np_param)

        px, py, yaw, speed, omega = x[0], x[1], x[2], x[3], x[4]
        accel, angular_accel = u[0], u[1]
        f_expl = ca.vertcat(
            speed * ca.cos(yaw), speed * ca.sin(yaw), omega, accel, angular_accel
        )

        xref = p[0:5]
        uref = p[5:7]
        yaw_error = ca.atan2(ca.sin(x[2] - xref[2]), ca.cos(x[2] - xref[2]))
        y_expr = ca.vertcat(
            x[0] - xref[0],
            x[1] - xref[1],
            yaw_error,
            x[3] - xref[3],
            x[4] - xref[4],
            u - uref,
        )
        y_expr_e = ca.vertcat(x[0] - xref[0], x[1] - xref[1], yaw_error, x[3] - xref[3])

        model = AcadosModel()
        model.name = self.config.model_name
        model.x = x
        model.xdot = xdot
        model.u = u
        model.p = p
        model.f_expl_expr = f_expl
        model.f_impl_expr = xdot - f_expl
        model.cost_y_expr = y_expr
        model.cost_y_expr_e = y_expr_e

        ocp = AcadosOcp()
        code_export_directory = (
            Path(self.config.code_export_directory)
            if self.config.code_export_directory
            else Path(__file__).resolve().parent / "results" / "acados_codegen"
        )
        code_export_directory.mkdir(parents=True, exist_ok=True)
        ocp.code_gen_options.code_export_directory = str(code_export_directory)
        ocp.code_gen_options.json_file = (
            self.config.json_file
            if self.config.json_file
            else str(code_export_directory / f"{self.config.model_name}_acados_ocp.json")
        )
        ocp.model = model
        ocp.name = self.config.model_name
        ocp.solver_options.N_horizon = self.config.steps
        ocp.solver_options.tf = self.config.horizon
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(
            np.concatenate(
                (
                    self.weights.position,
                    [self.weights.yaw, self.weights.speed, self.weights.omega],
                    self.weights.control,
                )
            )
        )
        ocp.cost.W_e = np.diag(
            np.concatenate((self.weights.terminal_position, [self.weights.terminal_yaw, self.weights.terminal_speed]))
        )
        ocp.cost.yref = np.zeros(7)
        ocp.cost.yref_e = np.zeros(4)
        ocp.constraints.x0 = np.zeros(nx)
        ocp.constraints.lbu = np.array([self.bounds.a_min, self.bounds.alpha_min])
        ocp.constraints.ubu = np.array([self.bounds.a_max, self.bounds.alpha_max])
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.idxbx = np.array([3, 4])
        ocp.constraints.lbx = np.array([self.bounds.v_min, self.bounds.omega_min])
        ocp.constraints.ubx = np.array([self.bounds.v_max, self.bounds.omega_max])
        ocp.constraints.idxbx_e = np.array([3, 4])
        ocp.constraints.lbx_e = np.array([self.bounds.v_min, self.bounds.omega_min])
        ocp.constraints.ubx_e = np.array([self.bounds.v_max, self.bounds.omega_max])
        ocp.parameter_values = np.zeros(np_param)
        ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        ocp.solver_options.qp_solver_cond_N = min(5, self.config.steps)
        ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        ocp.solver_options.integrator_type = "ERK"
        ocp.solver_options.nlp_solver_type = "SQP_RTI"
        ocp.solver_options.nlp_solver_max_iter = self.config.max_iter
        ocp.solver_options.nlp_solver_tol_stat = self.config.ftol
        ocp.solver_options.nlp_solver_tol_eq = self.config.ftol
        ocp.solver_options.nlp_solver_tol_ineq = self.config.ftol
        ocp.solver_options.nlp_solver_tol_comp = self.config.ftol
        ocp.solver_options.regularize_method = "CONVEXIFY"
        ocp.solver_options.print_level = 0
        return ocp, AcadosOcpSolver

    def export_solver_code(self) -> None:
        ocp, AcadosOcpSolver = self._build_ocp()
        AcadosOcpSolver.generate(ocp, verbose=False)
