import matplotlib.pyplot as plt
import pandas as pd
import sys
import matplotlib as mpl
import os
import glob
from matplotlib.animation import FuncAnimation

mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
mpl.rcParams['axes.unicode_minus'] = False

class AccuracyOnlyVisualizer:
    def __init__(self, csv_file, interval=100):
        """
        仅CSV准确率可视化类（带动画）
        参数:
            csv_file: CSV文件路径
            interval: 动画帧间隔（毫秒）
        """
        self.csv_file = csv_file
        self.interval = interval
        self.data = pd.DataFrame()
        self.n_points = 0
        self.load_data()

    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            
            # 将 time 列从 0 开始重新计数
            if 'time' in self.data.columns and not self.data.empty:
                self.data['time'] = self.data['time'] - self.data['time'].iloc[0]
                
            self.n_points = len(self.data)
            print(f"Successfully loaded data file: {self.csv_file}")
            print(f"Data rows: {self.n_points}")
            print(f"Data columns: {list(self.data.columns)}")
        except Exception as e:
            print(f"Failed to load CSV file: {e}")
            sys.exit(1)

    def setup_plots(self):
        """初始化图形和线条对象"""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(10, 8))

        # === 准确率图 ===
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        labels = ['Normal Data Accuracy', 'DOS Attack Accuracy', 'FDI Attack Accuracy', 'Replay Attack Accuracy']
        markers = ['', '', '', '']
        acc_keys = ['orix', 'oriy', 'oriz', 'oriw']

        self.acc_lines = {}
        for key, color, label, marker in zip(acc_keys, colors, labels, markers):
            line = self.ax.plot(
                [], [], label=label, linewidth=3, color=color,
                marker=marker, markersize=4, markerfacecolor='white',
                markeredgewidth=2, markeredgecolor=color, zorder=3
            )[0]
            self.acc_lines[key] = line

        self.ax.set_xlabel('Time (s)', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Accuracy (%)', fontsize=12, fontweight='bold')
        self.ax.set_title('UAV Accuracy Analysis', fontsize=16, pad=15, fontweight='bold')
        self.ax.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax.legend(loc='lower left', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)
        self.ax.set_facecolor('#f8f9fa')

        # 设置 y 轴范围
        if not self.data.empty:
            orix, oriy, oriz, oriw = self.data['orix'], self.data['oriy'], self.data['oriz'], self.data['oriw']
            min_acc = min(orix.min(), oriy.min(), oriz.min(), oriw.min())
            max_acc = max(orix.max(), oriy.max(), oriz.max(), oriw.max())
            padding = (max_acc - min_acc) * 0.02 if max_acc != min_acc else 0.01
            self.ax.set_ylim(max(0, min_acc - padding), min(100, max_acc + padding))

            # 设置 x 轴范围
            time_data = self.data['time'].values
            self.ax.set_xlim(time_data.min(), time_data.max())

        # 图表右上方悬浮的计数器框（在图表外部右上角）
        bbox_props = dict(boxstyle="round,pad=0.5", fc="white", ec="gray", alpha=0.9)
        self.total_text = self.fig.text(0.98, 0.96, f'Total Data Points: {self.n_points}', 
                                       ha='right', va='top', 
                                       fontsize=12, fontweight='bold',
                                       bbox=bbox_props, zorder=5)

    def animate(self, i):
        """动画更新函数"""
        if i >= self.n_points:
            return

        time_slice = self.data['time'].iloc[:i+1]

        # 更新准确率图
        self.acc_lines['orix'].set_data(time_slice, self.data['orix'].iloc[:i+1])
        self.acc_lines['oriy'].set_data(time_slice, self.data['oriy'].iloc[:i+1])
        self.acc_lines['oriz'].set_data(time_slice, self.data['oriz'].iloc[:i+1])
        self.acc_lines['oriw'].set_data(time_slice, self.data['oriw'].iloc[:i+1])

        # 更新当前绘制点数
        self.total_text.set_text(f'Current Time Point: {i+1} ')

        return list(self.acc_lines.values())


    def show(self):
        """显示动画"""
        self.setup_plots()
        self.anim = FuncAnimation(
            self.fig, self.animate, frames=self.n_points,
            interval=self.interval, blit=False, repeat=False
        )
        plt.tight_layout()
        plt.show()

def main():
    if len(sys.argv) < 2:
        print("Usage: python 单独准确率绘图.py <csv_file_path> [animation_interval_ms]")
        print("Example: python 单独准确率绘图.py AttackDetection_000.csv 100")
        sys.exit(1)

    csv_file = sys.argv[1]
    interval = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    visualizer = AccuracyOnlyVisualizer(csv_file, interval=interval)
    visualizer.show()

if __name__ == "__main__":
    main()
