import ctypes
import os
import sys
from importlib.util import find_spec
from pathlib import Path
from typing import List, Optional

import numpy as np


class Bounds:
    def __init__(
        self,
        a_min=-2.0,
        a_max=2.0,
        omega_min=-2.5,
        omega_max=2.5,
        v_min=-0.5,
        v_max=3.0,
    ):
        self.a_min = a_min
        self.a_max = a_max
        self.omega_min = omega_min
        self.omega_max = omega_max
        self.v_min = v_min
        self.v_max = v_max


class CostWeights:
    def __init__(
        self,
        position=None,
        yaw=8.0,
        speed=4.0,
        control=None,
        terminal_position=None,
        terminal_yaw=20.0,
        terminal_speed=10.0,
    ):
        self.position = np.array([20.0, 20.0]) if position is None else position
        self.yaw = yaw
        self.speed = speed
        self.control = np.array([0.05, 0.08]) if control is None else control
        self.terminal_position = (
            np.array([60.0, 60.0]) if terminal_position is None else terminal_position
        )
        self.terminal_yaw = terminal_yaw
        self.terminal_speed = terminal_speed


class MPCConfig:
    def __init__(
        self,
        horizon=1.0,
        steps=10,
        max_iter=20,
        ftol=1.0e-4,
        model_name="unicycle_nmpc",
        code_export_directory=None,
        json_file=None,
    ):
        self.horizon = horizon
        self.steps = steps
        self.max_iter = max_iter
        self.ftol = ftol
        self.model_name = model_name
        self.code_export_directory = code_export_directory
        self.json_file = json_file

    @property
    def dt(self) -> float:
        if self.steps <= 0:
            raise ValueError("steps must be positive")
        if self.horizon <= 0.0:
            raise ValueError("horizon must be positive")
        return self.horizon / self.steps


def _env_paths(*names: str) -> List[Path]:
    paths = []
    for name in names:
        for raw_path in os.environ.get(name, "").split(os.pathsep):
            if not raw_path:
                continue
            candidate = Path(raw_path).expanduser().resolve()
            if candidate.exists() and candidate not in paths:
                paths.append(candidate)
    return paths


def configure_acados_environment() -> None:
    os.environ.setdefault("MPLBACKEND", "Agg")
    acados_root = None
    for name in ("XGC2_ACADOS_SOURCE_DIR", "ACADOS_SOURCE_DIR", "ACADOS_ROOT"):
        raw = os.environ.get(name)
        if raw and Path(raw).expanduser().exists():
            acados_root = Path(raw).expanduser().resolve()
            break
    if acados_root is None and Path("/opt/xgc2/acados").exists():
        acados_root = Path("/opt/xgc2/acados")
    python_paths = _env_paths("XGC2_ACADOS_PYTHONPATH", "ACADOS_PYTHONPATH")
    if acados_root is not None:
        os.environ.setdefault("ACADOS_SOURCE_DIR", str(acados_root))
        template_parent = acados_root / "interfaces" / "acados_template"
        if template_parent.exists():
            python_paths.append(template_parent)
    for path in reversed(python_paths):
        path_str = str(path)
        if path_str not in sys.path:
            sys.path.insert(0, path_str)
    library_dirs = _env_paths("XGC2_ACADOS_LIBRARY_DIRS", "ACADOS_LIBRARY_DIRS")
    if acados_root is not None and (acados_root / "lib").exists():
        library_dirs.append(acados_root / "lib")
    for lib_dir in library_dirs:
        current_ld = os.environ.get("LD_LIBRARY_PATH", "")
        lib_dir_str = str(lib_dir)
        if lib_dir_str not in current_ld.split(":"):
            os.environ["LD_LIBRARY_PATH"] = f"{lib_dir_str}:{current_ld}" if current_ld else lib_dir_str
        for lib_name in ("libblasfeo.so", "libhpipm.so", "libacados.so"):
            lib_path = lib_dir / lib_name
            if lib_path.exists():
                ctypes.CDLL(str(lib_path), mode=ctypes.RTLD_GLOBAL)


class AcadosBackendUnavailable(RuntimeError):
    pass


class AcadosUnicycleNMPC:
    def __init__(
        self,
        config: Optional[MPCConfig] = None,
        bounds: Optional[Bounds] = None,
        weights: Optional[CostWeights] = None,
    ) -> None:
        configure_acados_environment()
        if find_spec("casadi") is None or find_spec("acados_template") is None:
            raise AcadosBackendUnavailable("casadi/acados_template is not importable")
        self.config = config if config is not None else MPCConfig()
        self.bounds = bounds if bounds is not None else Bounds()
        self.weights = weights if weights is not None else CostWeights()

    def _build_ocp(self):
        import casadi as ca
        from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

        nx = 4
        nu = 2
        np_param = nx + nu
        x = ca.SX.sym("x", nx)
        xdot = ca.SX.sym("xdot", nx)
        u = ca.SX.sym("u", nu)
        p = ca.SX.sym("p", np_param)

        px, py, yaw, speed = x[0], x[1], x[2], x[3]
        accel, omega = u[0], u[1]
        f_expl = ca.vertcat(speed * ca.cos(yaw), speed * ca.sin(yaw), omega, accel)

        xref = p[0:4]
        uref = p[4:6]
        yaw_error = ca.atan2(ca.sin(x[2] - xref[2]), ca.cos(x[2] - xref[2]))
        y_expr = ca.vertcat(x[0] - xref[0], x[1] - xref[1], yaw_error, x[3] - xref[3], u - uref)
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
        ocp.code_export_directory = str(code_export_directory)
        ocp.json_file = (
            self.config.json_file
            if self.config.json_file
            else str(code_export_directory / f"{self.config.model_name}_acados_ocp.json")
        )
        ocp.model = model
        ocp.solver_options.N_horizon = self.config.steps
        ocp.solver_options.tf = self.config.horizon
        ocp.cost.cost_type = "NONLINEAR_LS"
        ocp.cost.cost_type_e = "NONLINEAR_LS"
        ocp.cost.W = np.diag(
            np.concatenate((self.weights.position, [self.weights.yaw, self.weights.speed], self.weights.control))
        )
        ocp.cost.W_e = np.diag(
            np.concatenate((self.weights.terminal_position, [self.weights.terminal_yaw, self.weights.terminal_speed]))
        )
        ocp.cost.yref = np.zeros(6)
        ocp.cost.yref_e = np.zeros(4)
        ocp.constraints.x0 = np.zeros(nx)
        ocp.constraints.lbu = np.array([self.bounds.a_min, self.bounds.omega_min])
        ocp.constraints.ubu = np.array([self.bounds.a_max, self.bounds.omega_max])
        ocp.constraints.idxbu = np.array([0, 1])
        ocp.constraints.idxbx = np.array([3])
        ocp.constraints.lbx = np.array([self.bounds.v_min])
        ocp.constraints.ubx = np.array([self.bounds.v_max])
        ocp.constraints.idxbx_e = np.array([3])
        ocp.constraints.lbx_e = np.array([self.bounds.v_min])
        ocp.constraints.ubx_e = np.array([self.bounds.v_max])
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
        AcadosOcpSolver.generate(ocp, json_file=ocp.json_file, verbose=False)
