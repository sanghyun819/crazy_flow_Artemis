#!/usr/bin/env python3
"""Run TOGT on the measured seven gates and independently audit its polynomial output."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from togt_trajectory import Trajectory, audit, load_yaml


def export_track(course, settings, destination):
    hover = [course['start']['x'], course['start']['y'], course['start']['takeoff_z']]
    state = dict(pos=hover, vel=[0., 0., 0.], acc=[0., 0., 0.], jer=[0., 0., 0.],
                 rot=[1., 0., 0., 0.], euler=[0., 0., 0.], cthrustmass=settings['gravity_m_s2'])
    names = [f"Gate{gate['id']}" for gate in course['gates']]
    track = dict(initState=state.copy(), endState=state.copy(), orders=names)
    half = course['gate']['inner_size']/2
    reserve = settings['tracking_reserve_m']
    margin = 2*(course['gate']['drone_radius'] + course['gate']['frame_margin'] + reserve)
    if margin >= 2*half:
        raise ValueError('Drone envelope and clearances do not fit the aperture')
    for name, gate in zip(names, course['gates']):
        # TOGT entry is local +Z, exit local -Z. Set local +Z = -normal.
        track[name] = dict(type='RectanglePrisma', name='measured_cf_gate',
                           position=[gate['x'], gate['y'], gate['mount_z']+half],
                           rpy=[0., -90., -float(gate['yaw_deg'])], width=2*half, height=2*half,
                           marginW=margin, marginH=margin,
                           length=settings['gate_corridor_length_m'], midpoints=0, stationary=True)
    for i, guide in enumerate(settings.get('routing_points', []), 1):
        name = f'Bypass{i}'
        names.insert(names.index(f"Gate{guide['after_gate']}")+1, name)
        track[name] = dict(type='SingleBall', name='frame_bypass', position=guide['position'],
                           radius=guide['radius_m'], margin=0.0, stationary=True)
    # TOGT's YAML parser needs arrays inline. No anchors/aliases.
    lines = ['# Generated from seven_gates.yaml. CW yaw; entry -> exit is the forward normal.']
    for name, value in track.items():
        if isinstance(value, dict):
            lines.append(f'{name}:')
            for key, item in value.items():
                lines.append(f'  {key}: {json.dumps(item)}')
        else:
            lines.append(f'{name}: {json.dumps(value)}')
    destination.write_text('\n'.join(lines)+'\n', encoding='utf-8')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--togt-dir', type=Path, default=Path('/home/sh/togt_ws'))
    parser.add_argument('--gates', type=Path, default=ROOT/'config/seven_gates.yaml')
    parser.add_argument('--out', type=Path, default=ROOT/'results/seven_gates')
    parser.add_argument('--audit-only', action='store_true')
    args = parser.parse_args()
    course = load_yaml(args.gates)
    params = ROOT/'config/togt'
    settings, quad, limits = [load_yaml(params/name) for name in ['audit.yaml', 'quad.yaml', 'planning.yaml']]
    args.out.mkdir(parents=True, exist_ok=True)
    track_file = args.out/'track.yaml'
    raw_file = args.out/'togt_raw_polynomial.csv'
    if not args.audit_only:
        build = ROOT/'build'
        build.mkdir(exist_ok=True)
        binary = build/'togt_plan'
        source = ROOT/'togt/plan.cpp'
        if not binary.exists() or binary.stat().st_mtime < source.stat().st_mtime:
            subprocess.run(['c++', '-std=c++17', '-O2', f'-I{args.togt_dir}/include', '-I/usr/include/eigen3',
                            str(source), str(args.togt_dir/'build-core/libdrolib.a'), '-o', str(binary)], check=True)
        export_track(course, settings, track_file)
        with (args.out/'solver.log').open('w') as log:
            subprocess.run([str(binary), str(params), str(track_file), str(raw_file)], stdout=log,
                            stderr=subprocess.STDOUT, check=True)
    raw = Trajectory.load(raw_file)
    report, samples = audit(raw, course, quad, limits, settings)
    (args.out/'raw_audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    geometry_checks = [key for key in report['checks']
                       if not key.endswith('_sampled') and key != 'speed_limit']
    if not all(report['checks'][key] for key in geometry_checks):
        print(json.dumps(report, indent=2))
        raise SystemExit('Geometry verification failed; do not publish an executable trajectory.')
    factor = 1.0
    final = raw
    for _ in range(50):
        if report['passed']:
            break
        factor *= 1.03
        final = raw.scaled(factor)
        report, samples = audit(final, course, quad, limits, settings)
    if not report['passed']:
        raise SystemExit('Dynamic audit failed after time scaling')
    report['raw_duration_s'] = raw.duration
    report['time_scale'] = factor
    report['vehicle'] = quad
    report['settings'] = settings
    report['limits'] = limits
    report['source_course'] = course
    final.save(args.out/'togt_polynomial.csv')
    report['provenance'] = {
        'audit_time_utc': datetime.now(timezone.utc).isoformat(),
        'togt_git_revision': subprocess.check_output(
            ['git', '-C', str(args.togt_dir), 'rev-parse', 'HEAD'], text=True).strip(),
        'sha256': {str(path.relative_to(ROOT) if path.is_relative_to(ROOT) else path):
                   hashlib.sha256(path.read_bytes()).hexdigest()
                   for path in [args.gates, params/'quad.yaml', params/'planning.yaml', params/'audit.yaml',
                                track_file, raw_file, args.out/'togt_polynomial.csv']},
    }
    (args.out/'audit.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    header = 't,x,y,z,vx,vy,vz,ax,ay,az,jx,jy,jz,sx,sy,sz,qw,qx,qy,qz,wx,wy,wz,f1,f2,f3,f4'
    np.savetxt(args.out/'togt_samples.csv', samples, delimiter=',', header=header, comments='', fmt='%.10g')
    print(json.dumps({key: report[key] for key in ['passed', 'duration_s', 'length_m_sampled', 'pieces',
                                                 'max_speed_m_s', 'max_tilt_deg_sampled',
                                                 'rotor_thrust_min_N_sampled', 'rotor_thrust_max_N_sampled',
                                                 'vehicle_to_wall_gap_m', 'time_scale', 'crossings', 'frames']}, indent=2))


if __name__ == '__main__':
    main()
