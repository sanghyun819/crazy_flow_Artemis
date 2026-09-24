"""Validate and expand a safe gate-traversal route from YAML.

Each gate is expanded into an approach point, the aperture centre, and an exit
point.  The output is controller-neutral: it can be mapped to Crazyswarm
``goTo`` commands, MAVLink setpoints, or an offline trajectory generator.
"""
from __future__ import annotations

import json
import math
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any

import yaml


@dataclass(frozen=True)
class RoutePoint:
    label: str
    position: tuple[float, float, float]
    yaw_deg: float
    speed_m_s: float
    gate_center: bool = False


@dataclass(frozen=True)
class RouteLeg:
    target: RoutePoint
    distance_m: float
    duration_s: float


def _vector3(value: Any, name: str) -> tuple[float, float, float]:
    if not isinstance(value, (list, tuple)) or len(value) != 3:
        raise ValueError(f"{name} must be [x, y, z]")
    result = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in result):
        raise ValueError(f"{name} must contain finite values")
    return result


def _positive(mapping: dict[str, Any], key: str, section: str) -> float:
    try:
        value = float(mapping[key])
    except (KeyError, TypeError, ValueError) as error:
        raise ValueError(f"{section}.{key} must be a positive number") from error
    if not math.isfinite(value) or value <= 0.0:
        raise ValueError(f"{section}.{key} must be a positive number")
    return value


def load_route(path: Path) -> dict[str, Any]:
    """Read and validate a route document's structural requirements."""
    try:
        with path.open(encoding="utf-8") as file:
            route = yaml.safe_load(file)
    except OSError as error:
        raise ValueError(f"cannot read {path}: {error}") from error
    except yaml.YAMLError as error:
        raise ValueError(f"invalid YAML in {path}: {error}") from error
    if not isinstance(route, dict):
        raise ValueError("route must be a YAML mapping")
    if route.get("schema_version") != 1:
        raise ValueError("schema_version must be 1")
    if route.get("frame") != "world_z_up":
        raise ValueError("frame must be world_z_up")
    if not isinstance(route.get("vehicle"), dict) or not isinstance(route.get("route"), dict):
        raise ValueError("route needs vehicle and route mappings")
    if not isinstance(route.get("gates"), list) or not route["gates"]:
        raise ValueError("route needs at least one gate")
    _vector3(route.get("start"), "start")
    _vector3(route.get("finish"), "finish")
    return route


def _offset(center: tuple[float, float, float], yaw_deg: float,
            distance_m: float) -> tuple[float, float, float]:
    yaw_rad = math.radians(yaw_deg)
    return (
        center[0] + distance_m * math.cos(yaw_rad),
        center[1] + distance_m * math.sin(yaw_rad),
        center[2],
    )


def build_points(route: dict[str, Any], speed_scale: float = 1.0) -> list[RoutePoint]:
    """Validate clearance and expand every gate to entry, centre, and exit."""
    if not math.isfinite(speed_scale) or speed_scale <= 0.0:
        raise ValueError("speed_scale must be finite and greater than zero")
    vehicle = route["vehicle"]
    route_config = route["route"]
    safety_radius = _positive(vehicle, "safety_radius_m", "vehicle")
    approach = _positive(route_config, "approach_distance_m", "route")
    exit_distance = _positive(route_config, "exit_distance_m", "route")
    transit_speed = _positive(route_config, "transit_speed_m_s", "route") * speed_scale
    gate_speed = _positive(route_config, "gate_speed_m_s", "route") * speed_scale

    points = [RoutePoint("start", _vector3(route["start"], "start"), 0.0, transit_speed)]
    seen_ids: set[str] = set()
    for index, gate in enumerate(route["gates"], start=1):
        if not isinstance(gate, dict):
            raise ValueError(f"gates[{index - 1}] must be a mapping")
        gate_id = str(gate.get("id", f"Gate{index}"))
        if gate_id in seen_ids:
            raise ValueError(f"duplicate gate id: {gate_id}")
        seen_ids.add(gate_id)
        center = _vector3(gate.get("center"), f"{gate_id}.center")
        try:
            yaw_deg = float(gate["yaw_deg"])
            width = float(gate["inner_width_m"])
            height = float(gate["inner_height_m"])
        except (KeyError, TypeError, ValueError) as error:
            raise ValueError(f"{gate_id} needs center, yaw_deg, inner_width_m, inner_height_m") from error
        if not all(math.isfinite(value) for value in (yaw_deg, width, height)):
            raise ValueError(f"{gate_id} geometry must be finite")
        if width <= 2.0 * safety_radius or height <= 2.0 * safety_radius:
            raise ValueError(
                f"{gate_id} opening {width:.3f} x {height:.3f} m is too small for "
                f"safety radius {safety_radius:.3f} m")
        if center[2] - height / 2.0 < safety_radius:
            raise ValueError(f"{gate_id} lower edge violates configured ground clearance")
        points.extend((
            RoutePoint(f"{gate_id}: entry", _offset(center, yaw_deg, -approach), yaw_deg, transit_speed),
            RoutePoint(f"{gate_id}: center", center, yaw_deg, gate_speed, True),
            RoutePoint(f"{gate_id}: exit", _offset(center, yaw_deg, exit_distance), yaw_deg, transit_speed),
        ))
    points.append(RoutePoint("finish", _vector3(route["finish"], "finish"),
                             points[-1].yaw_deg, transit_speed))
    return points


def _distance(first: RoutePoint, second: RoutePoint) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(first.position, second.position)))


def build_legs(route: dict[str, Any], points: list[RoutePoint]) -> list[RouteLeg]:
    """Calculate conservative duration for each point-to-point controller command."""
    minimum = _positive(route["route"], "min_leg_duration_s", "route")
    legs: list[RouteLeg] = []
    previous = points[0]
    for target in points[1:]:
        distance = _distance(previous, target)
        duration = max(minimum, distance / min(previous.speed_m_s, target.speed_m_s))
        legs.append(RouteLeg(target, distance, duration))
        previous = target
    return legs


def summary(route: dict[str, Any], points: list[RoutePoint], legs: list[RouteLeg]) -> dict[str, Any]:
    """Return serialisable controller-neutral route information."""
    return {
        "frame": route["frame"],
        "gate_count": len(route["gates"]),
        "total_distance_m": round(sum(leg.distance_m for leg in legs), 3),
        "total_command_time_s": round(sum(leg.duration_s for leg in legs), 3),
        "points": [asdict(point) for point in points],
        "legs": [
            {
                "target": leg.target.label,
                "distance_m": round(leg.distance_m, 3),
                "duration_s": round(leg.duration_s, 3),
            }
            for leg in legs
        ],
    }


def print_plan(route: dict[str, Any], points: list[RoutePoint], legs: list[RouteLeg]) -> None:
    details = summary(route, points, legs)
    print(f"{details['gate_count']} gates | {details['total_distance_m']:.2f} m | "
          f"{details['total_command_time_s']:.1f} s")
    print(f"{'target':<17} {'x [m]':>7} {'y [m]':>7} {'z [m]':>7} {'yaw':>7} "
          f"{'leg [m]':>8} {'time [s]':>9}")
    print(f"{'start':<17} {points[0].position[0]:7.2f} {points[0].position[1]:7.2f} "
          f"{points[0].position[2]:7.2f} {points[0].yaw_deg:7.1f} {'-':>8} {'-':>9}")
    for leg in legs:
        point = leg.target
        print(f"{point.label:<17} {point.position[0]:7.2f} {point.position[1]:7.2f} "
              f"{point.position[2]:7.2f} {point.yaw_deg:7.1f} {leg.distance_m:8.2f} "
              f"{leg.duration_s:9.2f}")


def main() -> None:
    import argparse
    parser = argparse.ArgumentParser(description="Validate and expand a gate route")
    parser.add_argument("route", type=Path, help="route YAML")
    parser.add_argument("--speed-scale", type=float, default=1.0)
    parser.add_argument("--json", action="store_true", help="write route plan as JSON")
    args = parser.parse_args()
    try:
        route = load_route(args.route)
        points = build_points(route, args.speed_scale)
        legs = build_legs(route, points)
    except ValueError as error:
        raise SystemExit(f"invalid route: {error}") from error
    if args.json:
        print(json.dumps(summary(route, points, legs), indent=2))
    else:
        print_plan(route, points, legs)


if __name__ == "__main__":
    main()
