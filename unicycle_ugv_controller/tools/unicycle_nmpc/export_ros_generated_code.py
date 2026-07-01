from __future__ import annotations

import argparse
import shutil
from pathlib import Path

from .controller import AcadosUnicycleNMPC, MPCConfig


def clean_generated(output_dir: Path) -> None:
    stale_paths = [
        output_dir / "acados_solver_unicycle_nmpc.c",
        output_dir / "acados_solver_unicycle_nmpc.h",
        output_dir / "acados_sim_solver_unicycle_nmpc.c",
        output_dir / "acados_sim_solver_unicycle_nmpc.h",
        output_dir / "main_unicycle_nmpc.c",
        output_dir / "main_sim_unicycle_nmpc.c",
        output_dir / "unicycle_nmpc_model",
        output_dir / "unicycle_nmpc_cost",
        output_dir / "unicycle_nmpc_constraints",
    ]
    for path in stale_paths:
        if path.is_dir():
            shutil.rmtree(path)
        elif path.exists():
            path.unlink()


def remove_unused_export_files(output_dir: Path) -> None:
    for name in (
        "CMakeLists.txt",
        "Makefile",
        "main_unicycle_nmpc.c",
        "main_sim_unicycle_nmpc.c",
        "acados_sim_solver_unicycle_nmpc.c",
        "acados_sim_solver_unicycle_nmpc.h",
        "libacados_ocp_solver_unicycle_nmpc.so",
        "unicycle_nmpc_acados_ocp.json",
        "acados_solver.pxd",
    ):
        path = output_dir / name
        if path.exists():
            path.unlink()


def export_solver(output_dir: Path, *, horizon: float, steps: int) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    clean_generated(output_dir)
    json_file = output_dir / "unicycle_nmpc_acados_ocp.json"
    controller = AcadosUnicycleNMPC(
        config=MPCConfig(
            horizon=horizon,
            steps=steps,
            model_name="unicycle_nmpc",
            code_export_directory=str(output_dir),
            json_file=str(json_file),
        )
    )
    controller.export_solver_code()
    remove_unused_export_files(output_dir)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--horizon", type=float, default=1.0)
    parser.add_argument("--steps", type=int, default=10)
    args = parser.parse_args(argv)
    export_solver(args.output_dir, horizon=args.horizon, steps=args.steps)
    print(f"exported UGV unicycle acados solver to {args.output_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
