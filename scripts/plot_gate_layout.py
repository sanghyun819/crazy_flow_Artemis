#!/usr/bin/env python3
"""Draw gate geometry and CW normals; no trajectory generation or vehicle I/O."""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
from matplotlib.patches import Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
import yaml


ROOT = Path(__file__).resolve().parents[1]
COLORS = ['#2563eb', '#078b85', '#c77b08', '#7853b0',
          '#db5342', '#0d8eae', '#b7468c']
INK = '#152b42'


def gate_geometry(gate, size):
    angle = np.deg2rad(-gate['yaw_deg'])
    normal = np.array([np.cos(angle), np.sin(angle), 0.0])
    lateral = np.array([-normal[1], normal[0], 0.0])
    centre = np.array([gate['x'], gate['y'], gate['mount_z'] + size / 2])
    up = np.array([0., 0., 1.])
    corners = np.array([centre + u * size / 2 * lateral + v * size / 2 * up
                        for u, v in [(-1, -1), (1, -1), (1, 1), (-1, 1)]])
    return centre, normal, lateral, corners


def setup_style():
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.exists():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams.update({'font.size': 10, 'axes.unicode_minus': False,
                         'text.color': INK, 'axes.labelcolor': INK,
                         'xtick.color': '#536777', 'ytick.color': '#536777',
                         'axes.edgecolor': '#cad4de', 'figure.facecolor': '#f5f8fb',
                         'savefig.facecolor': '#f5f8fb'})


def draw_top(ax, data):
    room = data['room']
    hx, hy, margin = room['size_x'] / 2, room['size_y'] / 2, room['safety_margin']
    ax.set_facecolor('white')
    ax.add_patch(Rectangle((-hx, -hy), 2*hx, 2*hy, facecolor='#f0f3f7',
                           edgecolor=INK, lw=1.8, zorder=1))
    ax.add_patch(Rectangle((-hx+margin, -hy+margin), 2*(hx-margin), 2*(hy-margin),
                           facecolor='white', edgecolor='#8c9aaa', lw=1.4,
                           linestyle=(0, (5, 4)), zorder=2))
    for x in np.arange(-hx+1, hx):
        ax.plot([x, x], [-hy, hy], color='#e3eaf0', lw=.6, zorder=3)
    for y in np.arange(-hy+.5, hy, 1):
        ax.plot([-hx, hx], [y, y], color='#e3eaf0', lw=.6, zorder=3)
    ax.axhline(0, color='#b6c2ce', lw=.9, zorder=3)
    ax.axvline(0, color='#b6c2ce', lw=.9, zorder=3)
    offsets = {1: (32, 0), 2: (24, 23), 3: (30, 7), 4: (13, -26),
               5: (-86, 24), 6: (-86, -27), 7: (-75, -22)}
    size = data['gate']['inner_size']
    for gate, color in zip(data['gates'], COLORS):
        centre, normal, lateral, _ = gate_geometry(gate, size)
        ends = np.array([centre - size/2*lateral, centre + size/2*lateral])
        ax.plot(ends[:, 0], ends[:, 1], color=color,
                lw=5 if gate['id'] != 6 else 2.5,
                linestyle='-' if gate['id'] != 6 else '--', zorder=6)
        tip = centre + .72 * normal
        ax.annotate('', xy=tip[:2], xytext=centre[:2],
                    arrowprops={'arrowstyle': '-|>', 'color': color, 'lw': 2,
                                'mutation_scale': 16}, zorder=7)
        ax.scatter(*centre[:2], color=color, s=22, zorder=8)
        ax.annotate(f"G{gate['id']}  ·  z={centre[2]:.2f} m", xy=centre[:2],
                    xytext=offsets[gate['id']], textcoords='offset points',
                    color=color, fontsize=9.7, weight='bold', zorder=10,
                    bbox={'boxstyle': 'round,pad=.25', 'fc': 'white', 'ec': 'none', 'alpha': .95})
    start = data['start']
    ax.scatter(start['x'], start['y'], s=145, marker='*', color=INK, zorder=11)
    ax.annotate('이륙 / 착륙\n(1.5, -2.0)', (start['x'], start['y']),
                xytext=(20, -35), textcoords='offset points', fontsize=9.5,
                arrowprops={'arrowstyle': '-', 'color': '#627589'}, zorder=11)
    ax.text(0, -4.8, '수평 벽 여유 1.5 m', ha='center', color='#627589', fontsize=10)
    ax.set(xlim=(-hx-.25, hx+.25), ylim=(-hy-.25, hy+.25),
           xlabel='X [m]', ylabel='Y [m]')
    ax.set_xticks(np.arange(-4, 5, 1))
    ax.set_yticks(np.arange(-5, 6, 1))
    ax.set_aspect('equal')
    ax.set_title('01  상면도 · 방 중심이 원점', loc='left', fontsize=14, pad=14, weight='bold')
    ax.legend(handles=[Line2D([], [], color='#677d91', lw=4, label='게이트 개구부 평면'),
                       Line2D([], [], color='#8c9aaa', lw=1.5, ls='--', label='수평 여유 경계')],
              loc='lower left', frameon=True, facecolor='white', framealpha=1, fontsize=9)


def draw_3d(ax, data):
    ax.set_facecolor('white')
    size = data['gate']['inner_size']
    zlabels = {1: .10, 2: .12, 3: .10, 4: .10, 5: .12, 6: -.12, 7: .10}
    for gate, color in zip(data['gates'], COLORS):
        centre, normal, lateral, corners = gate_geometry(gate, size)
        closed = np.vstack([corners, corners[0]])
        ax.add_collection3d(Poly3DCollection([corners], facecolors=color, alpha=.07))
        ax.plot(*closed.T, color=color, lw=2.5)
        ax.quiver(*centre, *normal, length=.60, color=color, linewidth=1.7,
                  arrow_length_ratio=.27, normalize=True)
        ax.plot([centre[0]]*2, [centre[1]]*2, [0, corners[0, 2]],
                 color=color, alpha=.45, ls=':', lw=1)
        ax.scatter(centre[0], centre[1], 0, color=color, s=12, alpha=.55)
        # Labels sit just beyond the aperture's upper edge, except lower G6.
        label_z = centre[2] + (size/2 if gate['id'] != 6 else -size/2) + zlabels[gate['id']]
        label_x = centre[0] - .45 if gate['id'] == 6 else centre[0] + .04
        ax.text(label_x, centre[1], label_z, f"G{gate['id']}",
                 color=color, fontsize=10, weight='bold')
    start = data['start']
    ax.scatter(start['x'], start['y'], 0, marker='*', s=90, color=INK)
    ax.plot([start['x']]*2, [start['y']]*2, [0, start['takeoff_z']],
             color=INK, ls='--', lw=1.0)
    ax.scatter(start['x'], start['y'], start['takeoff_z'], color=INK, s=20)
    ax.set(xlim=(-2.25, 2.75), ylim=(-2.6, 2.9), zlim=(0, data['room']['height']),
           xlabel='X [m]', ylabel='Y [m]', zlabel='Z [m]')
    ax.set_box_aspect((5, 5.5, data['room']['height']))
    ax.set_zticks([0, .5, .75, 1.25, 2.01])
    ax.view_init(elev=26, azim=-62)
    for axis in (ax.xaxis, ax.yaxis, ax.zaxis):
        axis.pane.fill = False
        axis._axinfo['grid']['color'] = '#e3eaf0'
    ax.set_title('02  게이트 영역 확대 · 3D', loc='left', fontsize=14, weight='bold', pad=10)
    ax.text2D(.0, -.04, '선: 0.50 × 0.50 m 개구부 경계   |   화살표: 통과 방향',
              transform=ax.transAxes, fontsize=9, color='#627589')


def draw_table(ax, data):
    ax.axis('off')
    ax.set_title('03  설치 좌표 · 중심 높이', loc='left', fontsize=14, weight='bold', pad=9)
    rows = []
    for gate in data['gates']:
        rows.append([f"G{gate['id']}", f"{gate['x']:.2f}", f"{gate['y']:.2f}",
                     f"{gate['mount_z']:.2f}",
                     f"{gate['mount_z'] + data['gate']['inner_size']/2:.2f}",
                     f"{gate['yaw_deg']}°"])
    table = ax.table(cellText=rows,
                     colLabels=['게이트', 'X [m]', 'Y [m]', '하단 Z [m]', '중심 Z [m]', 'Yaw (CW)'],
                     colLoc='center', cellLoc='center', bbox=[0, .30, 1, .66])
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
    ax.text(0, .20, 'G5 · G6  |  같은 XY 위치, 상하 배치',
             transform=ax.transAxes, fontsize=12, weight='bold', color=INK)
    ax.text(0, .10, 'G5  중심 1.25 m · +Y 통과     /     G6  중심 0.75 m · −Y 통과',
             transform=ax.transAxes, fontsize=10.5, color='#536777')
    ax.text(0, .0, 'Yaw: +X 기준 시계방향   ·   방향 벡터 = (cos(−yaw), sin(−yaw))',
             transform=ax.transAxes, fontsize=9.5, color='#536777')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--gates', type=Path, default=ROOT / 'config' / 'seven_gates.yaml')
    parser.add_argument('--output', type=Path, default=ROOT / 'docs' / 'seven-gates-layout.png')
    args = parser.parse_args()
    with args.gates.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream)
    setup_style()
    fig = plt.figure(figsize=(16, 11), dpi=180)
    grid = fig.add_gridspec(2, 2, left=.055, right=.965, bottom=.095, top=.84,
                           width_ratios=[1, 1.16], height_ratios=[1.22, 1],
                           wspace=.12, hspace=.20)
    fig.text(.055, .945, '7개 게이트 배치도', fontsize=26, weight='bold', color=INK)
    fig.text(.055, .903,
             '방 8.0 × 11.0 × 2.01 m   |   개구부 0.50 × 0.50 m   |   좌표계: 방 중심 원점 · Z 위쪽',
             fontsize=12, color='#536777')
    draw_top(fig.add_subplot(grid[:, 0]), data)
    draw_3d(fig.add_subplot(grid[0, 1], projection='3d'), data)
    draw_table(fig.add_subplot(grid[1, 1]), data)
    fig.text(.055, .040,
             '사용자가 제공한 gates.yaml 기준  ·  이륙 위치 (1.5, −2.0), 호버 Z=1.25 m  ·  비행 경로 미포함',
             fontsize=10, color='#536777')
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180)
    fig.savefig(args.output.with_suffix('.svg'))
    plt.close(fig)
    print(f'Saved: {args.output}')
    print(f'Saved: {args.output.with_suffix(".svg")}')


if __name__ == '__main__':
    main()
