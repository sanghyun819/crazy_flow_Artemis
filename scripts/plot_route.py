#!/usr/bin/env python3
"""Create a top-view and altitude-profile PNG for a gate-route YAML."""
from __future__ import annotations

import argparse
import math
from pathlib import Path
import sys

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from gate_route import build_legs, build_points, load_route  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Visualize a gate route")
    parser.add_argument("route", type=Path, nargs="?", default=ROOT / "config" / "demo_gate_route.yaml")
    parser.add_argument("--output", type=Path, default=ROOT / "docs" / "route-preview.png")
    return parser.parse_args()


def draw_gate(axis, gate: dict, safety_radius: float) -> None:
    x, y, _ = gate["center"]
    yaw = math.radians(float(gate["yaw_deg"]))
    tangent_x, tangent_y = -math.sin(yaw), math.cos(yaw)
    half_width = float(gate["inner_width_m"]) / 2.0
    endpoints = ((x - tangent_x * half_width, y - tangent_y * half_width),
                 (x + tangent_x * half_width, y + tangent_y * half_width))
    axis.plot(*zip(*endpoints), color="#e4572e", linewidth=8, solid_capstyle="butt", zorder=2)
    axis.plot(*zip(*endpoints), color="#fff8e7", linewidth=5, solid_capstyle="butt", zorder=3)
    axis.arrow(x, y, 0.24 * math.cos(yaw), 0.24 * math.sin(yaw), width=0.016,
               head_width=0.09, head_length=0.09, color="#e4572e", length_includes_head=True, zorder=5)
    axis.text(x, y + 0.16, gate["id"], ha="center", va="bottom", fontsize=10,
              fontweight="bold", color="#98351c")
    axis.text(x, y - 0.16, f"{gate['inner_width_m']:.1f} × {gate['inner_height_m']:.1f} m\nr={safety_radius:.2f} m",
              ha="center", va="top", fontsize=7, color="#555555")


def main() -> None:
    args = parse_args()
    route = load_route(args.route)
    points = build_points(route)
    legs = build_legs(route, points)
    safety_radius = float(route["vehicle"]["safety_radius_m"])
    args.output.parent.mkdir(parents=True, exist_ok=True)

    figure, (top_view, altitude) = plt.subplots(2, 1, figsize=(11, 11),
        gridspec_kw={"height_ratios": (4, 1)}, constrained_layout=True)
    xs = [point.position[0] for point in points]
    ys = [point.position[1] for point in points]
    top_view.plot(xs, ys, "--", color="#2d70b3", linewidth=1.8, zorder=1)
    for gate in route["gates"]:
        draw_gate(top_view, gate, safety_radius)
    for point in points:
        x, y, _ = point.position
        color, size, marker = ("#2d70b3", 35, "o")
        if point.label == "start":
            color, size, marker = "#202020", 180, "*"
        elif point.label == "finish":
            color, size, marker = "#202020", 60, "s"
        elif point.gate_center:
            color, size = "#c23b22", 55
        elif point.label.endswith("exit"):
            color = "#3a9d5d"
        top_view.scatter(x, y, marker=marker, s=size, color=color, zorder=7)
    top_view.set(title="Gate route — top view (world frame, +Z up)", xlabel="x [m]", ylabel="y [m]")
    top_view.set_aspect("equal", adjustable="box")
    top_view.grid(True, color="#dddddd")
    top_view.legend(handles=[
        Line2D([], [], color="#2d70b3", linestyle="--", label="commanded path"),
        Line2D([], [], color="#e4572e", linewidth=8, label="gate / travel direction"),
        Line2D([], [], color="#2d70b3", marker="o", linestyle="None", label="entry"),
        Line2D([], [], color="#c23b22", marker="o", linestyle="None", label="gate centre"),
        Line2D([], [], color="#3a9d5d", marker="o", linestyle="None", label="exit"),
    ], loc="upper right", framealpha=0.96)

    cumulative_distance = [0.0]
    for leg in legs:
        cumulative_distance.append(cumulative_distance[-1] + leg.distance_m)
    z_values = [point.position[2] for point in points]
    altitude.plot(cumulative_distance, z_values, color="#6a4c93", marker="o")
    altitude.fill_between(cumulative_distance, z_values, 0, color="#6a4c93", alpha=0.12)
    altitude.set(title="Altitude profile", xlabel="path distance [m]", ylabel="z [m]",
                 ylim=(0, max(z_values) + 0.4))
    altitude.grid(True, color="#dddddd")
    figure.suptitle(f"{len(route['gates'])} gates | {cumulative_distance[-1]:.2f} m | "
                     f"command time {sum(leg.duration_s for leg in legs):.1f} s", fontsize=11, y=1.02)
    figure.savefig(args.output, dpi=180, bbox_inches="tight")
    print(f"saved {args.output}")


if __name__ == "__main__":
    main()
