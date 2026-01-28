#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
import matplotlib as mpl
from mpl_toolkits.mplot3d.art3d import Line3DCollection, Poly3DCollection

mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
mpl.rcParams['axes.unicode_minus'] = False

class TrajectoryVisualizer:
    def __init__(self, csv_files, interval=200):
        self.csv_files = csv_files
        self.interval = interval
        self.trajectories = []
        self.n_frames = 0
        self.load_data()

    def load_data(self):
        """加载所有轨迹数据"""
        for idx, filename in enumerate(self.csv_files):
            traj = self.read_trajectory(filename)
            if traj.size == 0:
                print(f"警告：文件 {filename} 中没有有效的轨迹数据，已跳过。")
                continue
            self.trajectories.append(traj)
            print(f"成功加载轨迹 {idx}: {filename}, 点数: {len(traj)}")
        
        if not self.trajectories:
            print("错误：没有有效的轨迹数据")
            sys.exit(1)
        
        self.n_frames = max([len(d) for d in self.trajectories])
        print(f"总轨迹数: {len(self.trajectories)}, 最大帧数: {self.n_frames}")

    def read_trajectory(self, file_path):
        points = []
        with open(file_path, 'r') as f:
            for line in f:
                parts = line.strip().split(',')
                if len(parts) < 3:
                    continue
                try:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    points.append([x, y, z])
                except ValueError:
                    continue
        return np.array(points)

    def setup_plots(self):
        self.fig = plt.figure(figsize=(12, 9))
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig.suptitle('IRIS 无人机动态 3D 轨迹图', fontsize=16, fontweight='bold')

        cmap = plt.get_cmap('tab20')

        # 存储每条轨迹的线条和飞机图标
        self.lines = []           # 虚线轨迹
        self.aircrafts = []       # 飞机图标（由两个部分组成：机身+尾翼）
        self.start_markers = []   # 起点标记

        for idx, traj in enumerate(self.trajectories):
            color = cmap(idx % 20)
            # 创建虚线轨迹
            line = self.ax.plot([], [], [], color=color, linewidth=2, linestyle='--', label=f'iris{idx}')[0]
            self.lines.append(line)

            # 创建起点标记（圆圈）
            start_marker = self.ax.scatter([], [], [], color=color, s=80, marker='o', edgecolors='k', zorder=5)
            self.start_markers.append(start_marker)

            # 创建飞机图标（简化为一个箭头+机身）
            aircraft = self.create_aircraft(traj[0], color, scale=1.0)
            self.aircrafts.append(aircraft)

        # 设置坐标轴标签
        self.ax.set_xlabel('X', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y', fontsize=12, fontweight='bold')
        self.ax.set_zlabel('Z', fontsize=12, fontweight='bold')
        self.ax.set_title('3D 轨迹动画', fontsize=14, pad=20, fontweight='bold')
        self.ax.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax.legend(title="无人机", loc='best', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)

        # 设置坐标轴范围
        all_x = np.concatenate([traj[:, 0] for traj in self.trajectories])
        all_y = np.concatenate([traj[:, 1] for traj in self.trajectories])
        all_z = np.concatenate([traj[:, 2] for traj in self.trajectories])
        
        self.ax.set_xlim(all_x.min(), all_x.max())
        self.ax.set_ylim(all_y.min(), all_y.max())
        self.ax.set_zlim(all_z.min(), all_z.max())

        # 总数据点数文本
        self.total_text = self.fig.text(0.5, 0.90, f'轨迹数：{len(self.trajectories)}', 
                                        ha='center', fontsize=12, fontweight='bold')

    def create_aircraft(self, pos, color, scale=1.0):
        """创建一个简化的飞机图标（机身+尾翼）"""
        # 机身：从尾到头的箭头
        body_length = 0.5 * scale
        wing_width = 0.2 * scale
        tail_length = 0.3 * scale

        # 坐标系：以 pos 为中心，方向为 (dx, dy, dz)，我们假设飞行方向为正 Z 方向（可扩展）
        dx, dy, dz = 0, 0, 1  # 默认朝前（Z 正方向），实际应根据速度向量计算
        # 简化处理：仅使用当前位置，方向固定为 Z 轴

        # 机身（长方体）
        body_points = [
            [pos[0] - wing_width/2, pos[1], pos[2] - body_length],
            [pos[0] + wing_width/2, pos[1], pos[2] - body_length],
            [pos[0] + wing_width/2, pos[1], pos[2]],
            [pos[0] - wing_width/2, pos[1], pos[2]]
        ]
        body = Poly3DCollection([body_points], facecolor=color, edgecolor='black', alpha=0.8)
        body.set_facecolor(color)
        body.set_edgecolor('black')
        body.set_alpha(0.8)
        self.ax.add_collection3d(body)

        # 尾翼（小三角形）
        tail_points = [
            [pos[0], pos[1] - wing_width/2, pos[2] - body_length],
            [pos[0], pos[1] + wing_width/2, pos[2] - body_length],
            [pos[0], pos[1], pos[2] - body_length - tail_length]
        ]
        tail = Poly3DCollection([tail_points], facecolor=color, edgecolor='black', alpha=0.8)
        tail.set_facecolor(color)
        tail.set_edgecolor('black')
        tail.set_alpha(0.8)
        self.ax.add_collection3d(tail)

        return body, tail

    def animate(self, i):
        """动画更新函数"""
        for line, traj, start_marker, aircraft in zip(self.lines, self.trajectories, 
                                                     self.start_markers, self.aircrafts):
            # 更新轨迹线（虚线）
            line.set_data(traj[:i, 0], traj[:i, 1])
            line.set_3d_properties(traj[:i, 2])

            # 更新起点标记
            if i > 0:
                start_marker._offsets3d = ([traj[0, 0]], [traj[0, 1]], [traj[0, 2]])

            # 更新飞机图标位置（只更新最后一个点）
            if i > 0:
                current_pos = traj[i-1]
                # 删除旧图标（需重新创建）
                for obj in aircraft:
                    try:
                        obj.remove()
                    except (ValueError, AttributeError):
                        pass
                new_aircraft = self.create_aircraft(current_pos, line.get_color())
                # 更新aircraft引用
                idx = self.aircrafts.index(aircraft)
                self.aircrafts[idx] = new_aircraft

        return self.lines + self.start_markers

    def show(self):
        """显示动画"""
        self.setup_plots()
        self.anim = FuncAnimation(
            self.fig, self.animate, frames=self.n_frames,
            interval=self.interval, blit=False, repeat=False
        )
        plt.tight_layout()
        plt.subplots_adjust(top=0.90, bottom=0.10)
        plt.show()

def main():
    if len(sys.argv) < 2:
        print("使用方法: python 3d轨迹.py <csv文件1> <csv文件2> ... [动画间隔毫秒]")
        print("示例: python 3d.py iris_0_data.csv iris_2_data.csv 200")
        sys.exit(1)

    csv_files = sys.argv[1:-1] if len(sys.argv) > 2 and sys.argv[-1].isdigit() else sys.argv[1:]
    interval = int(sys.argv[-1]) if len(sys.argv) > 1 and sys.argv[-1].isdigit() else 200

    visualizer = TrajectoryVisualizer(csv_files, interval=interval)
    visualizer.show()

if __name__ == "__main__":
    main()