"""Evaluate and audit TOGT polynomial trajectories independently of the solver.

Position bounds, speed maximum and gate-plane intersections use polynomial roots.
Frame distances are sampled with a conservative Lipschitz gap correction.
Flatness dynamics are evaluated at 1 kHz, with fixed pre-tilt yaw = 0.
"""
from pathlib import Path
import math
import numpy as np
from numpy.polynomial import polynomial as poly
import yaml


def load_yaml(path):
    with Path(path).open(encoding='utf-8') as stream:
        return yaml.safe_load(stream)


def real_unit_roots(coefficients):
    c = np.asarray(coefficients, dtype=float)
    # Normalize coefficient scale to reduce sensitivity for low-amplitude polynomials.
    if np.max(np.abs(c)) < 1e-14:
        return np.array([])
    roots = poly.polyroots(poly.polytrim(c / np.max(np.abs(c)), tol=1e-13))
    return np.clip([r.real for r in roots
                    if abs(r.imag) < 1e-7 and -1e-8 <= r.real <= 1+1e-8], 0, 1)


class Trajectory:
    def __init__(self, values):
        values = np.atleast_2d(values)
        if values.shape[1] != 33 or not np.all(np.isfinite(values)):
            raise ValueError('Expected finite 33-column polynomial CSV')
        if np.any(values[:, 0] <= 0):
            raise ValueError('Piece durations must be positive')
        self.values = values
        self.durations = values[:, 0]
        self.coefficients = values[:, 1:25].reshape(-1, 3, 8)
        self.times = np.r_[0, np.cumsum(self.durations)]
        self.duration = float(self.times[-1])

    @classmethod
    def load(cls, path):
        return cls(np.loadtxt(path, delimiter=',', skiprows=1))

    def save(self, path):
        header = 'duration,' + ','.join(f'{a}^{k}' for a in ('x', 'y', 'z', 'yaw') for k in range(8))
        np.savetxt(path, self.values, delimiter=',', header=header, comments='', fmt='%.17g')

    def scaled(self, factor):
        values = self.values.copy()
        values[:, 0] *= factor
        for axis in range(4):
            values[:, 1+8*axis:9+8*axis] /= factor ** np.arange(8)
        return Trajectory(values)

    def evaluate(self, times, derivative=0):
        times = np.atleast_1d(times)
        indices = np.minimum(np.searchsorted(self.times[1:], times, side='right'),
                             len(self.durations)-1)
        answer = np.empty((len(times), 3))
        for i in np.unique(indices):
            mask = indices == i
            coefficients = poly.polyder(self.coefficients[i], m=derivative, axis=1)
            answer[mask] = poly.polyval(times[mask]-self.times[i], coefficients.T).T
        return answer

    def exact_bounds(self):
        low, high = np.full(3, np.inf), np.full(3, -np.inf)
        max_speed = 0.
        for duration, coefficients in zip(self.durations, self.coefficients):
            unit_c = coefficients * duration ** np.arange(8)
            for axis in range(3):
                roots = real_unit_roots(poly.polyder(unit_c[axis]))
                values = poly.polyval(np.r_[0., roots, 1.], unit_c[axis])
                low[axis] = min(low[axis], values.min())
                high[axis] = max(high[axis], values.max())
            unit_v = poly.polyder(unit_c, axis=1) / duration
            speed_squared = np.array([0.])
            for v in unit_v:
                speed_squared = poly.polyadd(speed_squared, poly.polymul(v, v))
            roots = real_unit_roots(poly.polyder(speed_squared))
            max_speed = max(max_speed, math.sqrt(max(0., poly.polyval(np.r_[0., roots, 1.], speed_squared).max())))
        return low, high, max_speed


def gate_axes(gate, size):
    psi = math.radians(-gate['yaw_deg'])
    normal = np.array([math.cos(psi), math.sin(psi), 0.])
    lateral = np.array([-normal[1], normal[0], 0.])
    centre = np.array([gate['x'], gate['y'], gate['mount_z']+size/2])
    return centre, normal, lateral


def gate_events(trajectory, course):
    events = []
    half = course['gate']['inner_size']/2
    for gate in course['gates']:
        centre, normal, lateral = gate_axes(gate, 2*half)
        previous_time = -1.
        for i, (duration, coefficients) in enumerate(zip(trajectory.durations, trajectory.coefficients)):
            signed = normal @ coefficients
            signed[0] -= normal @ centre
            roots = sorted(real_unit_roots(signed * duration ** np.arange(8)))
            for root in roots:
                t = trajectory.times[i] + root*duration
                if abs(t-previous_time) < 1e-6:
                    continue
                previous_time = t
                position = trajectory.evaluate([t])[0]
                lateral_error = float((position-centre) @ lateral)
                vertical_error = float(position[2]-centre[2])
                if max(abs(lateral_error), abs(vertical_error)) <= half+1e-7:
                    events.append({'gate': int(gate['id']), 'time_s': float(t),
                                   'position_m': position.tolist(),
                                   'normal_speed_m_s': float(trajectory.evaluate([t], 1)[0] @ normal),
                                   'lateral_offset_m': lateral_error,
                                   'vertical_offset_m': vertical_error,
                                   'edge_to_vehicle_m': half-max(abs(lateral_error), abs(vertical_error))
                                                        -course['gate']['drone_radius']})
    return sorted(events, key=lambda event: event['time_s'])


def frame_clearance(positions, course, frame_depth, gap_error):
    half = course['gate']['inner_size']/2
    width = course['gate']['frame_thickness']
    outer = half+width
    # Coordinates: forward normal, lateral, vertical. Four solid frame bars.
    boxes = [([-frame_depth/2, -outer, -outer], [frame_depth/2, -half, outer]),
             ([-frame_depth/2, half, -outer], [frame_depth/2, outer, outer]),
             ([-frame_depth/2, -half, -outer], [frame_depth/2, half, -half]),
             ([-frame_depth/2, -half, half], [frame_depth/2, half, outer])]
    results = []
    for gate in course['gates']:
        centre, normal, lateral = gate_axes(gate, 2*half)
        delta = positions-centre
        local = np.column_stack([delta @ normal, delta @ lateral, delta[:, 2]])
        distances = np.full(len(positions), np.inf)
        for low, high in boxes:
            outside = np.maximum(np.maximum(np.asarray(low)-local, local-np.asarray(high)), 0.)
            distances = np.minimum(distances, np.linalg.norm(outside, axis=1))
        index = int(np.argmin(distances))
        results.append({'gate': int(gate['id']), 'sample_index': index,
                        'sampled_surface_gap_m': float(distances[index]-course['gate']['drone_radius']),
                        'conservative_surface_gap_m': float(distances[index]-course['gate']['drone_radius']-gap_error)})
    return results


def flatness_dynamics(acc, jerk, snap, quad, gravity):
    force_mass = acc + [0, 0, gravity]
    magnitude = np.linalg.norm(force_mass, axis=1)
    z = force_mass / magnitude[:, None]
    dot_magnitude = np.einsum('ij,ij->i', z, jerk)
    dz = (jerk - z*dot_magnitude[:, None])/magnitude[:, None]
    ddmag = (np.einsum('ij,ij->i', jerk, jerk)-dot_magnitude**2)/magnitude + np.einsum('ij,ij->i', z, snap)
    ddz = (snap-z*ddmag[:, None]-2*dz*dot_magnitude[:, None])/magnitude[:, None]
    den = 1+z[:, 2]
    wx = -dz[:, 1]+z[:, 1]*dz[:, 2]/den
    wy = dz[:, 0]-z[:, 0]*dz[:, 2]/den
    cross = z[:, 1]*dz[:, 0]-z[:, 0]*dz[:, 1]
    wz = cross/den
    omega = np.column_stack([wx, wy, wz])
    dot_wx = -ddz[:, 1]+(dz[:, 1]*dz[:, 2]+z[:, 1]*ddz[:, 2])/den-z[:, 1]*dz[:, 2]**2/den**2
    dot_wy = ddz[:, 0]-(dz[:, 0]*dz[:, 2]+z[:, 0]*ddz[:, 2])/den+z[:, 0]*dz[:, 2]**2/den**2
    dot_wz = (z[:, 1]*ddz[:, 0]-z[:, 0]*ddz[:, 1])/den-cross*dz[:, 2]/den**2
    inertia = np.asarray(quad['inertia'])
    torque = inertia*np.column_stack([dot_wx, dot_wy, dot_wz]) + np.cross(omega, omega*inertia)
    wrench = np.column_stack([quad['mass']*magnitude, torque])
    rotor = wrench @ np.linalg.inv(quad['T_bm']).T
    tilt = np.arccos(np.clip(z[:, 2], -1, 1))
    qden = np.sqrt(2*den)
    quaternion = np.column_stack([qden/2, -z[:, 1]/qden, z[:, 0]/qden, np.zeros(len(z))])
    return rotor, omega, tilt, quaternion


def audit(trajectory, course, quad, limits, settings):
    times = np.linspace(0, trajectory.duration, math.ceil(trajectory.duration/settings['audit_step_s'])+1)
    # Audit both sides of every polynomial join as well as the uniform grid.
    knots = trajectory.times[1:-1]
    times = np.unique(np.r_[times, knots, np.nextafter(knots, -np.inf)])
    states = [trajectory.evaluate(times, derivative) for derivative in range(5)]
    pos, vel, acc, jerk, snap = states
    low, high, vmax = trajectory.exact_bounds()
    rotor, omega, tilt, quaternion = flatness_dynamics(acc, jerk, snap, quad, settings['gravity_m_s2'])
    events = gate_events(trajectory, course)
    step = float(np.diff(times).max())
    gap_correction = vmax*step/2
    frames = frame_clearance(pos, course, settings['frame_depth_m'], gap_correction)
    radius = course['gate']['drone_radius']
    wall_gap = min(course['room']['size_x']/2-max(-low[0], high[0]),
                   course['room']['size_y']/2-max(-low[1], high[1]))-radius
    start = np.array([course['start']['x'], course['start']['y'], course['start']['takeoff_z']])
    continuity = []
    for derivative in range(4):
        worst = 0.
        for i in range(len(trajectory.durations)-1):
            left = poly.polyval(trajectory.durations[i],
                                poly.polyder(trajectory.coefficients[i], derivative, axis=1).T)
            right = poly.polyder(trajectory.coefficients[i+1], derivative, axis=1)[:, 0]
            worst = max(worst, float(np.linalg.norm(left-right)))
        continuity.append(worst)
    checks = {
        'finite_states': bool(all(np.all(np.isfinite(x)) for x in [*states, rotor, omega, tilt, quaternion])),
        'start_and_finish_match': bool(np.max(np.linalg.norm(pos[[0, -1]]-start, axis=1)) < 1e-6),
        'rest_endpoints': bool(all(np.max(np.abs(state[[0, -1]])) < 1e-5 for state in states[1:4])),
        'C3_continuity': bool(max(continuity) < 1e-5),
        'gate_order_once_each': [e['gate'] for e in events] == [g['id'] for g in course['gates']],
        'gate_direction_forward': all(e['normal_speed_m_s'] > 1e-5 for e in events),
        'gate_aperture_clearance': all(e['edge_to_vehicle_m'] >= course['gate']['frame_margin'] for e in events),
        'all_frames_clearance': all(f['conservative_surface_gap_m'] >= course['gate']['frame_margin'] for f in frames),
        'horizontal_wall_margin': bool(wall_gap >= course['room']['safety_margin']),
        'floor_ceiling_clearance': bool(low[2]-radius >= .10 and course['room']['height']-high[2]-radius >= .10),
        'planner_position_bounds': bool(all(low[k] >= limits['bound'+axis][0]-1e-6 and high[k] <= limits['bound'+axis][1]+1e-6
                                           for k, axis in enumerate('XYZ'))),
        'speed_limit': bool(vmax <= limits['maxVelNorm']),
        'tilt_limit_sampled': bool(tilt.max() <= limits['maxTiltedAngle']),
        'bodyrate_limit_sampled': bool(np.linalg.norm(omega[:, :2], axis=1).max() <= limits['maxOmgXY'] and np.abs(omega[:, 2]).max() <= limits['maxOmgZ']),
        'rotor_thrust_planning_limit_sampled': bool(rotor.min() >= limits['minThr'] and rotor.max() <= limits['maxThr']),
        'rotor_thrust_hardware_limit_sampled': bool(rotor.min() >= settings['hardware_rotor_thrust_min_N'] and rotor.max() <= settings['hardware_rotor_thrust_max_N']),
    }
    report = {'passed': all(checks.values()), 'checks': checks,
              'duration_s': trajectory.duration, 'pieces': len(trajectory.durations),
              'length_m_sampled': float(np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()),
              'position_min_m': low.tolist(), 'position_max_m': high.tolist(),
              'max_speed_m_s': vmax, 'max_acceleration_m_s2_sampled': float(np.linalg.norm(acc, axis=1).max()),
              'max_tilt_deg_sampled': float(np.rad2deg(tilt.max())),
              'max_bodyrate_xy_rad_s_sampled': float(np.linalg.norm(omega[:, :2], axis=1).max()),
              'max_bodyrate_z_rad_s_sampled': float(np.abs(omega[:, 2]).max()),
              'rotor_thrust_min_N_sampled': float(rotor.min()), 'rotor_thrust_max_N_sampled': float(rotor.max()),
              'rotor_min_each_N_sampled': rotor.min(axis=0).tolist(), 'rotor_max_each_N_sampled': rotor.max(axis=0).tolist(),
              'vehicle_to_wall_gap_m': float(wall_gap), 'vehicle_to_floor_gap_m': float(low[2]-radius),
              'vehicle_to_ceiling_gap_m': float(course['room']['height']-high[2]-radius),
              'max_join_error_pvaj': continuity, 'crossings': events, 'frames': frames,
              'audit_step_s': step, 'frame_gap_inter_sample_bound_m': gap_correction,
              'notes': ['Polynomial roots: bounds, gate-plane crossings, maximum speed.',
                        'Frame clearance lower bound = sample minimum - vehicle radius - max_speed * max_dt / 2.',
                        'Dynamics extrema are sampled, not certified continuous-time bounds.',
                        'Rigid body, no drag or motor lag; fixed pre-tilt yaw 0; gravity matches compiled TOGT.',
                        'Frame depth is an explicit 0.08 m modelling assumption; drone envelope is a sphere.']}
    # Samples are fully recomputed from the final timed polynomials.
    samples = np.column_stack([times, pos, vel, acc, jerk, snap, quaternion, omega, rotor])
    return report, samples
