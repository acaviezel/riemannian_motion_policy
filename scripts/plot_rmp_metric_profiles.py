#!/usr/bin/env python3
"""Plot force and position RMP metric profiles from a controller YAML config.

This script mirrors the current formulas used in the controller:
- Position controller metric from the attractor term.
- Force controller metric from the sigmoid-based force primitive.
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, Tuple

import matplotlib.pyplot as plt
import numpy as np
import yaml


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "src" / "config.yaml"


def parse_vector(text: str, expected_size: int, name: str) -> np.ndarray:
    values = [float(v.strip()) for v in text.split(",") if v.strip()]
    if len(values) != expected_size:
        raise ValueError(f"{name} must contain exactly {expected_size} comma-separated values.")
    return np.array(values, dtype=float)


def normalize(vec: np.ndarray, name: str) -> np.ndarray:
    norm = np.linalg.norm(vec)
    if norm < 1e-12:
        raise ValueError(f"{name} must be non-zero.")
    return vec / norm


def orthogonal_unit(vec: np.ndarray) -> np.ndarray:
    basis_candidate = np.array([1.0, 0.0, 0.0], dtype=float)
    if abs(np.dot(vec, basis_candidate)) > 0.9:
        basis_candidate = np.array([0.0, 1.0, 0.0], dtype=float)
    orth = basis_candidate - np.dot(basis_candidate, vec) * vec
    return normalize(orth, "orthogonal direction")


def position_metric_from_distance(
    distance_m: float,
    task_direction: np.ndarray,
    attractor_cfg: Dict,
) -> np.ndarray:
    alpha_min = float(attractor_cfg.get("alpha_min", 0.001))
    sigma_a = max(float(attractor_cfg.get("sigma_a", 0.005)), 1e-12)
    sigma_b = max(float(attractor_cfg.get("sigma_b", 0.008)), 1e-12)
    b_value = float(attractor_cfg.get("b", 30.0))
    weight = float(attractor_cfg.get("weight_attractor", 1.0))
    enabled = 1.0 if bool(attractor_cfg.get("enabled", True)) else 0.0

    sq_norm = float(distance_m * distance_m)
    alpha = (1.0 - alpha_min) * np.exp(-sq_norm / (2.0 * sigma_a)) + alpha_min
    beta = np.exp(-sq_norm / (2.0 * sigma_b))

    m_near = np.eye(3)
    m_far = np.outer(task_direction, task_direction)
    gain = beta * b_value + (1.0 - beta)

    return enabled * weight * gain * (alpha * m_near + (1.0 - alpha) * m_far)


def force_metric_from_contact(
    contact_force: float,
    joint_direction: np.ndarray,
    force_cfg: Dict,
) -> Tuple[np.ndarray, np.ndarray]:
    low = max(0.0, float(force_cfg.get("low_torque_threshold", 1.0)))
    high = max(low + 1e-6, float(force_cfg.get("high_torque_threshold", 5.0)))
    max_inertia = float(force_cfg.get("max_inertia", 5.0))
    steepness = float(force_cfg.get("sigmoid_steepness", 10.0))
    weight = float(force_cfg.get("weight_force_controller", 1.0))
    enabled = 1.0 if bool(force_cfg.get("enabled", True)) else 0.0

    midpoint = 0.5 * (low + high)
    slope = steepness / (high - low)

    # Proxy mapping: project increasing contact-force magnitude along one joint-space direction.
    tau_ext = contact_force * joint_direction
    activation = 1.0 / (1.0 + np.exp(-slope * (np.abs(tau_ext) - midpoint)))

    diagonal = enabled * weight * max_inertia * activation
    return np.diag(diagonal), tau_ext


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Plot RMP metric profiles from YAML config.")
    parser.add_argument(
        "--config",
        type=Path,
        default=DEFAULT_CONFIG,
        help=f"Path to config.yaml used by the controller (default: {DEFAULT_CONFIG}).",
    )
    parser.add_argument(
        "--distance-max-cm",
        type=float,
        default=5.0,
        help="Maximum distance to target in cm (profile runs from this value to 0).",
    )
    parser.add_argument(
        "--force-max",
        type=float,
        default=None,
        help="Maximum contact-force magnitude for the force profile.",
    )
    parser.add_argument(
        "--samples",
        type=int,
        default=300,
        help="Number of points per profile.",
    )
    parser.add_argument(
        "--task-direction",
        type=str,
        default="1,1,1",
        help="Arbitrary 3D task-space direction as comma-separated values.",
    )
    parser.add_argument(
        "--joint-direction",
        type=str,
        default="1,1,1,1,1,1,1",
        help="Arbitrary 7D joint-space direction as comma-separated values.",
    )
    parser.add_argument(
        "--save",
        type=Path,
        default=None,
        help="Optional output path for the figure (for example metrics.png).",
    )
    parser.add_argument(
        "--no-show",
        action="store_true",
        help="Do not open a window, only save or print summary.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()

    if not args.config.exists():
        raise FileNotFoundError(
            f"Config file not found at {args.config}. Pass --config to override the path."
        )

    with args.config.open("r", encoding="utf-8") as f:
        config = yaml.safe_load(f)

    attractor_cfg = dict(config.get("attractor", {}))
    force_cfg = dict(config.get("force_controller", {}))

    task_direction = normalize(parse_vector(args.task_direction, 3, "task-direction"), "task-direction")
    task_orthogonal = orthogonal_unit(task_direction)
    joint_direction = normalize(
        parse_vector(args.joint_direction, 7, "joint-direction"),
        "joint-direction",
    )

    distance_max_m = max(0.0, args.distance_max_cm) / 100.0
    distances = np.linspace(distance_max_m, 0.0, max(2, args.samples))

    low = max(0.0, float(force_cfg.get("low_torque_threshold", 1.0)))
    high = max(low + 1e-6, float(force_cfg.get("high_torque_threshold", 5.0)))
    if args.force_max is None:
        force_max = max(5.0, 2.5 * high)
    else:
        force_max = max(0.0, args.force_max)
    contact_forces = np.linspace(0.0, force_max, max(2, args.samples))

    m_pos_along = []
    m_pos_orth = []
    for distance in distances:
        metric = position_metric_from_distance(distance, task_direction, attractor_cfg)
        m_pos_along.append(float(task_direction @ metric @ task_direction))
        m_pos_orth.append(float(task_orthogonal @ metric @ task_orthogonal))

    m_force_along = []
    m_force_avg = []
    for force in contact_forces:
        metric, _ = force_metric_from_contact(force, joint_direction, force_cfg)
        m_force_along.append(float(joint_direction @ metric @ joint_direction))
        m_force_avg.append(float(np.trace(metric) / 7.0))

    fig, axes = plt.subplots(1, 2, figsize=(13, 5.2))

    axes[0].plot(distances * 100.0, m_pos_along, label="u^T M_pos u", linewidth=2.0)
    axes[0].plot(distances * 100.0, m_pos_orth, label="u_perp^T M_pos u_perp", linestyle="--", linewidth=2.0)
    axes[0].invert_xaxis()
    axes[0].set_xlabel("Distance to target [cm]")
    axes[0].set_ylabel("Position metric value")
    axes[0].set_title("Position Controller Metric vs Distance")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend()

    axes[1].plot(contact_forces, m_force_along, label="v^T M_force v", linewidth=2.0)
    axes[1].plot(contact_forces, m_force_avg, label="trace(M_force)/7", linestyle="--", linewidth=2.0)
    axes[1].axvline(low, color="gray", linestyle=":", linewidth=1.0, label="low threshold")
    axes[1].axvline(high, color="black", linestyle=":", linewidth=1.0, label="high threshold")
    axes[1].set_xlabel("Contact force magnitude proxy")
    axes[1].set_ylabel("Force metric value")
    axes[1].set_title("Force Controller Metric vs Contact Force")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend()

    fig.suptitle(f"RMP Metrics from {args.config.name}")
    fig.tight_layout()

    if args.save is not None:
        fig.savefig(args.save, dpi=180)
        print(f"Saved figure to {args.save}")

    if not args.no_show:
        plt.show()


if __name__ == "__main__":
    main()
