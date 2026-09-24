#!/usr/bin/env python3
"""Render the audited TOGT reference, physical gates, and timing diagnostics."""
import argparse
import json
from pathlib import Path
import sys

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle, Circle
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from togt_trajectory import Trajectory, gate_axes
from plot_gate_layout import setup_style, draw_3d, COLORS, INK


def top_view(ax, course, report, positions):
    ax.set_facecolor('white')
    room = course['room']
    hx = room['size_x']/2-room['safety_margin']
    hy = room['size_y']/2-room['safety_margin']
    ax.add_patch(Rectangle((-hx, -hy), 2*hx, 2*hy, fill=False, color='#8c9aaa',
                           ls='--', lw=1.2))
    ax.plot(positions[:, 0], positions[:, 1], color='#18456d', lw=1.8, zorder=3)
    # Arrows follow actual sampled velocity, not straight lines between gates.
    for k in np.linspace(150, len(positions)-100, 14, dtype=int):
        ax.annotate('', xy=positions[k+14, :2], xytext=positions[k, :2],
                    arrowprops={'arrowstyle': '-|>', 'color': '#18456d', 'lw': 1.4,
                                'mutation_scale': 13}, zorder=4)
    offsets = {1: (28, -15), 2: (0, 30), 3: (25, 5), 4: (12, -24),
               5: (-72, 22), 6: (-72, -29), 7: (-70, -16)}
    for gate, color in zip(course['gates'], COLORS):
        c, n, lateral = gate_axes(gate, course['gate']['inner_size'])
        h = course['gate']['inner_size']/2
        endpoints = np.array([c-h*lateral, c+h*lateral])
        ax.plot(endpoints[:, 0], endpoints[:, 1], color=color,
                lw=5 if gate['id'] != 6 else 2.5,
                ls='-' if gate['id'] != 6 else '--', zorder=6)
        ax.annotate('', xy=(c+.37*n)[:2], xytext=c[:2],
                    arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 1.8}, zorder=7)
        ax.annotate(f"G{gate['id']} · {c[2]:.2f} m", c[:2], xytext=offsets[gate['id']],
                    textcoords='offset points', color=color, fontsize=9.5, weight='bold',
                    bbox={'boxstyle': 'round,pad=.2', 'fc': 'white', 'ec': 'none', 'alpha': .93}, zorder=9)
    start = positions[0]
    ax.scatter(*start[:2], color=INK, marker='*', s=140, zorder=8)
    ax.annotate('시작 / 종료', start[:2], xytext=(18, -25), textcoords='offset points', fontsize=10)
    for guide in report['settings'].get('routing_points', []):
        x, y, _ = guide['position']
        ax.add_patch(Circle((x, y), guide['radius_m'], fill=False, color='#7d8b9b', lw=1.2, zorder=4))
        ax.annotate('G4 프레임 우회', (x, y), xytext=(35, 5), textcoords='offset points',
                    color='#687e91', fontsize=8.5, arrowprops={'arrowstyle': '-', 'color': '#aab6c2'})
    ax.set(xlim=(-2.85, 2.85), ylim=(-2.65, 3.05), xlabel='X [m]', ylabel='Y [m]')
    ax.set_aspect('equal')
    ax.grid(color='#dfe7ef', lw=.7)
    ax.set_title('01  TOGT 경로 · 상면도', loc='left', fontsize=14, weight='bold', pad=12)
    ax.legend(handles=[Line2D([], [], color='#18456d', label='검증된 연속 궤적'),
                       Line2D([], [], color='#8c9aaa', ls='--', label='벽 여유 1.5 m 경계')],
              loc='lower left', framealpha=.98, fontsize=9)


def diagnostics(ax, trajectory, report, times, positions):
    speed = np.linalg.norm(trajectory.evaluate(times, 1), axis=1)
    ax.plot(times, positions[:, 2], color='#7853b0', lw=2, label='고도 Z [m]')
    ax.plot(times, speed, color='#16889c', lw=1.7, label='속도 [m/s]')
    ax.axhline(1.5, color='#16889c', ls='--', lw=1, alpha=.5)
    for event, color in zip(report['crossings'], COLORS):
        ax.axvline(event['time_s'], color=color, alpha=.25, lw=1)
        ax.text(event['time_s'], 1.72, f"G{event['gate']}", color=color, ha='center', fontsize=9, weight='bold')
    ax.set(xlim=(0, trajectory.duration), ylim=(0, 1.88), xlabel='시간 [s]', ylabel='Z [m] / 속도 [m/s]')
    ax.grid(color='#dfe7ef', lw=.7)
    ax.legend(loc='lower left', fontsize=9, framealpha=.95)
    ax.set_title('03  통과 시각 · 고도 · 속도', loc='left', fontsize=14, weight='bold', pad=12)


def validation_table(ax, report):
    ax.axis('off')
    ax.set_title('04  게이트 통과 · 전체 경로 프레임 여유', loc='left', fontsize=14, weight='bold', pad=12)
    rows = []
    for event, frame in zip(report['crossings'], report['frames']):
        rows.append([f"G{event['gate']}", f"{event['time_s']:.2f} s", '정방향',
                     f"{frame['conservative_surface_gap_m']*100:.1f} cm"])
    table = ax.table(cellText=rows, colLabels=['게이트', '통과 시각', '통과 방향', '최소 프레임 여유'],
                     loc='center', colLoc='center', cellLoc='center', bbox=[0, .22, 1, .72])
    table.auto_set_font_size(False)
    table.set_fontsize(10)
    for (row, col), cell in table.get_celld().items():
        cell.set_edgecolor('#e3eaf0')
        cell.set_linewidth(.6)
        if row == 0:
            cell.set_facecolor(INK)
            cell.set_text_props(color='white', weight='bold')
        else:
            cell.set_facecolor('white' if row % 2 else '#ecf2f7')
            if col == 0:
                cell.set_text_props(color=COLORS[row-1], weight='bold')
    ax.text(0, .10, f"모터별 추력 {report['rotor_thrust_min_N_sampled']:.3f}–{report['rotor_thrust_max_N_sampled']:.3f} N"
                    f"  /  모델 상한 0.120 N", fontsize=11, color=INK, transform=ax.transAxes)
    ax.text(0, -.01, f"최대 틸트 {report['max_tilt_deg_sampled']:.1f}°  ·  벽 여유 {report['vehicle_to_wall_gap_m']:.2f} m"
                     f"  ·  천장 여유 {report['vehicle_to_ceiling_gap_m']:.2f} m", fontsize=10.5,
             color='#536777', transform=ax.transAxes)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--result', type=Path, default=ROOT/'results/seven_gates')
    parser.add_argument('--output', type=Path, default=ROOT/'docs/seven-gates-togt-path.png')
    args = parser.parse_args()
    report = json.loads((args.result/'audit.json').read_text())
    if not report['passed']:
        raise SystemExit('Refusing to label an unverified reference as validated')
    course = report['source_course']
    trajectory = Trajectory.load(args.result/'togt_polynomial.csv')
    times = np.linspace(0, trajectory.duration, 3000)
    positions = trajectory.evaluate(times)
    setup_style()
    fig = plt.figure(figsize=(17, 12), dpi=180)
    grid = fig.add_gridspec(2, 2, left=.065, right=.95, bottom=.135, top=.845,
                           height_ratios=[1.55, 1], wspace=.22, hspace=.31)
    gap = min(f['conservative_surface_gap_m'] for f in report['frames'])
    fig.text(.065, .952, '7개 게이트 · TOGT 경로', fontsize=26, weight='bold', color=INK)
    fig.text(.065, .914, f"{trajectory.duration:.2f} s   /   {report['length_m_sampled']:.2f} m   /   "
                         f"최대 속도 {report['max_speed_m_s']:.2f} m/s   /   프레임 최소 여유 {gap*100:.1f} cm",
             fontsize=13, color=INK)
    fig.text(.065, .879, 'Crazyflie 31.9 g · 반경 6 cm + 프레임 여유 10 cm · G1 → G7 → 시작점 복귀',
             fontsize=11, color='#536777')
    top_view(fig.add_subplot(grid[0, 0]), course, report, positions)
    ax3d = fig.add_subplot(grid[0, 1], projection='3d')
    draw_3d(ax3d, course)
    ax3d.plot(*positions.T, color='#18456d', lw=1.6)
    ax3d.set_title('02  연속 궤적 · 3D', loc='left', fontsize=14, weight='bold')
    # The descent after G5 and reverse-direction traversal of G6 are visible here.
    ax3d.set_ylim(-2.6, 3.0)
    diagnostics(fig.add_subplot(grid[1, 0]), trajectory, report, times, positions)
    validation_table(fig.add_subplot(grid[1, 1]), report)
    fig.text(.065, .065,
             f"TOGT 7차 다항식 {report['pieces']}구간 · 시간 배율 {report['time_scale']:.2f} · 고정 yaw 0 · "
             '통과/경계: 다항식 근 검증 · 동역학: 1 kHz 검증', fontsize=10, color='#536777')
    fig.text(.065, .038,
             '여유: 기체 반경을 뺀 거리 · 프레임 깊이 0.08 m 가정 · 강체 모델(항력·모터 지연 제외) · 오프라인 계획 결과',
             fontsize=9.5, color='#536777')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    fig.savefig(args.output.with_suffix('.svg'))
    plt.close(fig)
    print(args.output)


if __name__ == '__main__':
    main()
