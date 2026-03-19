#!/usr/bin/env python3
import sys
import os
import glob
import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d import Axes3D

# 论文风格设置
plt.rcParams.update({
    'font.family': 'serif',
    'font.serif': ['Times New Roman'],
    'font.size': 12,
    'axes.linewidth': 1.0,
    'axes.labelsize': 13,
    'axes.titlesize': 14,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'legend.fontsize': 11,
    'figure.dpi': 150,
    'savefig.dpi': 300,
    'savefig.bbox': 'tight',
})


def read_trajectory(file_path):
    """读取CSV轨迹文件"""
    points = []
    for encoding in ('utf-8', 'gbk'):
        try:
            with open(file_path, 'r', encoding=encoding) as f:
                for line in f:
                    parts = line.strip().split(',')
                    if len(parts) < 3:
                        continue
                    try:
                        points.append([float(parts[0]), float(parts[1]), float(parts[2])])
                    except ValueError:
                        continue
            break
        except UnicodeDecodeError:
            continue
    return np.array(points)


def plot_trajectories(csv_files):
    """一次性静态绘制所有3D轨迹（论文风格）"""
    # 读取数据
    trajectories = []
    for f in csv_files:
        traj = read_trajectory(f)
        if traj.size == 0:
            print(f"Warning: {f} 无有效数据，已跳过")
            continue
        trajectories.append(traj)
        print(f"已加载: {f}, 共 {len(traj)} 个点")

    if not trajectories:
        print("Error: 无有效轨迹数据")
        sys.exit(1)

    # 论文常用配色（偏柔和、可区分）
    colors = [
        '#1f77b4',  # 蓝
        '#d62728',  # 红
        '#2ca02c',  # 绿
        '#ff7f0e',  # 橙
        '#9467bd',  # 紫
        '#8c564b',  # 棕
        '#e377c2',  # 粉
        '#17becf',  # 青
    ]
    linestyles = ['-', '--', '-.', ':']
    markers = ['o', 's', '^', 'D', 'v', 'p', '*', 'h']

    # 创建图形
    fig = plt.figure(figsize=(8, 6))
    ax = fig.add_subplot(111, projection='3d')

    # 白底设置
    fig.patch.set_facecolor('white')
    ax.set_facecolor('white')

    # 绘制每条轨迹
    for idx, traj in enumerate(trajectories):
        color = colors[idx % len(colors)]
        ls = linestyles[idx % len(linestyles)]
        mk = markers[idx % len(markers)]

        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2],
                color=color, linewidth=1.5, linestyle='-', alpha=0.7,
                label=f'UAV-{idx}', zorder=2)

        # 起点标记
        ax.scatter(traj[0, 0], traj[0, 1], traj[0, 2],
                   color=color, marker=mk, s=60, edgecolors='black',
                   linewidth=0.8, zorder=3)

        # 终点标记（用 × 表示）
        ax.scatter(traj[-1, 0], traj[-1, 1], traj[-1, 2],
                   color=color, marker='x', s=80, linewidth=2, zorder=3)

    # 坐标轴标签
    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')

    # 网格线
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.5)

    # 坐标面板透明度
    ax.xaxis.pane.set_alpha(0.05)
    ax.yaxis.pane.set_alpha(0.05)
    ax.zaxis.pane.set_alpha(0.05)

    # 图例
    ax.legend(loc='upper right', framealpha=0.9, edgecolor='gray')

    # 标题
    ax.set_title('3D Trajectory Visualization', pad=15)

    plt.tight_layout()
    plt.savefig('3d_trajectories.png', dpi=300, bbox_inches='tight',
                facecolor='white', edgecolor='none')
    print("图片已保存: 3d_trajectories.png")
    plt.show()


def main():
    # 自动检测脚本所在目录下 iris 开头的 csv 文件
    script_dir = os.path.dirname(os.path.abspath(__file__))
    csv_files = sorted(glob.glob(os.path.join(script_dir, 'iris*.csv')))

    if not csv_files:
        print(f"Error: 在 {script_dir} 下未找到 iris 开头的 CSV 文件")
        sys.exit(1)

    print(f"检测到 {len(csv_files)} 个文件: {[os.path.basename(f) for f in csv_files]}")
    plot_trajectories(csv_files)


if __name__ == "__main__":
    main()