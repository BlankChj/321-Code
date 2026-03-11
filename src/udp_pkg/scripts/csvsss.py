import matplotlib.pyplot as plt
import pandas as pd
import sys
import matplotlib as mpl
from matplotlib.animation import FuncAnimation

mpl.rcParams['font.sans-serif'] = ['SimHei', 'Microsoft YaHei', 'Arial Unicode MS', 'DejaVu Sans']
mpl.rcParams['axes.unicode_minus'] = False

class CSVAccuracyVisualizer:
    def __init__(self, csv_file, interval=100):
        """
        CSV准确率可视化类（带动画）
        参数:
            csv_file: CSV文件路径
            interval: 动画帧间隔（毫秒）
        """
        self.csv_file = csv_file
        self.interval = interval
        self.data = None
        self.n_points = 0
        self.load_data()

    def load_data(self):
        """加载CSV数据"""
        try:
            self.data = pd.read_csv(self.csv_file)
            self.n_points = len(self.data)
            print(f"成功加载数据文件: {self.csv_file}")
            print(f"数据行数: {self.n_points}")
            print(f"数据列: {list(self.data.columns)}")
        except Exception as e:
            print(f"加载CSV文件失败: {e}")
            sys.exit(1)

    def setup_plots(self):
        """初始化图形和线条对象"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(20, 8))
        self.fig.suptitle('无人机位置与准确率分析', fontsize=16, fontweight='bold')

        # === 左图：位置 ===
        self.pos_lines = {
            'posx': self.ax1.plot([], [], label='posx', linewidth=2, color='#A6D8FF', marker='', markersize=4)[0],
            'posy': self.ax1.plot([], [], label='posy', linewidth=2, color='#FF9999', marker='', markersize=4)[0],
            'posz': self.ax1.plot([], [], label='posz', linewidth=2, color='#99FF96', marker='', markersize=4)[0],
        }
        self.ax1.set_xlabel('时间', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('位置', fontsize=12, fontweight='bold')
        self.ax1.set_title('无人机位置变化', fontsize=14, pad=20, fontweight='bold')
        self.ax1.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax1.legend(loc='upper right', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)
        self.ax1.set_facecolor('#f8f9fa')

        # 设置 x 轴范围（避免动画抖动）
        time_data = self.data['time'].values
        self.ax1.set_xlim(time_data.min(), time_data.max())
        pos_min = min(self.data['posx'].min(), self.data['posy'].min(), self.data['posz'].min())
        pos_max = max(self.data['posx'].max(), self.data['posy'].max(), self.data['posz'].max())
        padding = (pos_max - pos_min) * 0.05
        self.ax1.set_ylim(pos_min - padding, pos_max + padding)

        # === 右图：准确率 ===
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        labels = ['正常数据准确率', 'DOS攻击准确率', 'FDI攻击准确率', 'Replay攻击准确率']
        markers = ['', '', '', '']
        acc_keys = ['orix', 'oriy', 'oriz', 'oriw']

        self.acc_lines = {}
        for key, color, label, marker in zip(acc_keys, colors, labels, markers):
            line = self.ax2.plot(
                [], [], label=label, linewidth=3, color=color,
                marker=marker, markersize=4, markerfacecolor='white',
                markeredgewidth=2, markeredgecolor=color, zorder=3
            )[0]
            self.acc_lines[key] = line

        self.ax2.set_xlabel('时间', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('准确率 (%)', fontsize=12, fontweight='bold')
        self.ax2.set_title('准确率变化', fontsize=14, pad=20, fontweight='bold')
        self.ax2.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax2.legend(loc='lower right', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)
        self.ax2.set_facecolor('#f8f9fa')

        # 设置 y 轴范围
        orix, oriy, oriz, oriw = self.data['orix'], self.data['oriy'], self.data['oriz'], self.data['oriw']
        min_acc = min(orix.min(), oriy.min(), oriz.min(), oriw.min())
        max_acc = max(orix.max(), oriy.max(), oriz.max(), oriw.max())
        padding = (max_acc - min_acc) * 0.02 if max_acc != min_acc else 0.01
        self.ax2.set_ylim(max(0, min_acc - padding), min(100, max_acc + padding))

        # 设置 x 轴范围
        self.ax2.set_xlim(time_data.min(), time_data.max())

        # 总数据点数文本
        self.total_text = self.fig.text(0.5, 0.90, f'数据点数：{self.n_points}个', ha='center', fontsize=12, fontweight='bold')

    def animate(self, i):
        """动画更新函数"""
        if i >= self.n_points:
            return

        time_slice = self.data['time'].iloc[:i+1]
        
        # 更新位置图
        self.pos_lines['posx'].set_data(time_slice, self.data['posx'].iloc[:i+1])
        self.pos_lines['posy'].set_data(time_slice, self.data['posy'].iloc[:i+1])
        self.pos_lines['posz'].set_data(time_slice, self.data['posz'].iloc[:i+1])

        # 更新准确率图
        self.acc_lines['orix'].set_data(time_slice, self.data['orix'].iloc[:i+1])
        self.acc_lines['oriy'].set_data(time_slice, self.data['oriy'].iloc[:i+1])
        self.acc_lines['oriz'].set_data(time_slice, self.data['oriz'].iloc[:i+1])
        self.acc_lines['oriw'].set_data(time_slice, self.data['oriw'].iloc[:i+1])

        # 更新当前绘制点数
        self.total_text.set_text(f'当前时间点：{i+1} ')

        return list(self.pos_lines.values()) + list(self.acc_lines.values())


    def show(self):
        """显示动画"""
        self.setup_plots()
        self.anim = FuncAnimation(
            self.fig, self.animate, frames=self.n_points,
            interval=self.interval, blit=False, repeat=False
        )
        plt.tight_layout()
        plt.subplots_adjust(top=0.90, bottom=0.10)
        plt.show()

def main():
    if len(sys.argv) < 2:
        print("使用方法: python csv准确率绘图.py <csv文件路径> [动画间隔毫秒]")
        print("示例: python csv准确率绘图.py AttackDetection_000.csv 100")
        sys.exit(1)

    csv_file = sys.argv[1]
    interval = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    visualizer = CSVAccuracyVisualizer(csv_file, interval=interval)
    visualizer.show()

if __name__ == "__main__":
    main()