import matplotlib.pyplot as plt
import pandas as pd
import sys
from matplotlib.animation import FuncAnimation



class CSVAccuracyVisualizer:
    def __init__(self, csv_file, interval=100):

        self.csv_file = csv_file
        self.interval = interval
        self.data = None
        self.n_points = 0
        self.load_data()

    def load_data(self):
        """Load CSV data"""
        try:
            self.data = pd.read_csv(self.csv_file)
            self.n_points = len(self.data)
            print(f"Successfully loaded data file: {self.csv_file}")
            print(f"Data rows: {self.n_points}")
            print(f"Data columns: {list(self.data.columns)}")
        except Exception as e:
            print(f"Failed to load CSV file: {e}")
            sys.exit(1)

    def setup_plots(self):
        """Initialize figure and line objects"""
        self.fig, (self.ax1, self.ax2) = plt.subplots(1, 2, figsize=(20, 8))
        self.fig.suptitle('UAV Position and Accuracy Analysis', fontsize=16, fontweight='bold')

        # === Left plot: Position ===
        self.pos_lines = {
            'posx': self.ax1.plot([], [], label='posx', linewidth=2, color='#A6D8FF', marker='', markersize=4)[0],
            'posy': self.ax1.plot([], [], label='posy', linewidth=2, color='#FF9999', marker='', markersize=4)[0],
            'posz': self.ax1.plot([], [], label='posz', linewidth=2, color='#99FF96', marker='', markersize=4)[0],
        }
        self.ax1.set_xlabel('Time', fontsize=12, fontweight='bold')
        self.ax1.set_ylabel('Position', fontsize=12, fontweight='bold')
        self.ax1.set_title('UAV Position Change', fontsize=14, pad=20, fontweight='bold')
        self.ax1.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax1.legend(loc='upper right', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)
        self.ax1.set_facecolor('#f8f9fa')

        # Set x-axis range (avoid animation jitter)
        time_data = self.data['time'].values
        self.ax1.set_xlim(time_data.min(), time_data.max())
        pos_min = min(self.data['posx'].min(), self.data['posy'].min(), self.data['posz'].min())
        pos_max = max(self.data['posx'].max(), self.data['posy'].max(), self.data['posz'].max())
        padding = (pos_max - pos_min) * 0.05
        self.ax1.set_ylim(pos_min - padding, pos_max + padding)

        # === Right plot: Accuracy ===
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
        labels = ['Normal Data Accuracy', 'DOS Attack Accuracy', 'FDI Attack Accuracy', 'Replay Attack Accuracy']
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

        self.ax2.set_xlabel('Time', fontsize=12, fontweight='bold')
        self.ax2.set_ylabel('Accuracy (%)', fontsize=12, fontweight='bold')
        self.ax2.set_title('Accuracy Change', fontsize=14, pad=20, fontweight='bold')
        self.ax2.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax2.legend(loc='lower right', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)
        self.ax2.set_facecolor('#f8f9fa')

        # Set y-axis range
        orix, oriy, oriz, oriw = self.data['orix'], self.data['oriy'], self.data['oriz'], self.data['oriw']
        min_acc = min(orix.min(), oriy.min(), oriz.min(), oriw.min())
        max_acc = max(orix.max(), oriy.max(), oriz.max(), oriw.max())
        padding = (max_acc - min_acc) * 0.02 if max_acc != min_acc else 0.01
        self.ax2.set_ylim(max(0, min_acc - padding), min(100, max_acc + padding))

        # Set x-axis range
        self.ax2.set_xlim(time_data.min(), time_data.max())

        # Total data points text
        self.total_text = self.fig.text(0.5, 0.90, f'Data points: {self.n_points}', ha='center', fontsize=12, fontweight='bold')

    def animate(self, i):
        """Animation update function"""
        if i >= self.n_points:
            return

        time_slice = self.data['time'].iloc[:i+1]
        
        # Update position plot
        self.pos_lines['posx'].set_data(time_slice, self.data['posx'].iloc[:i+1])
        self.pos_lines['posy'].set_data(time_slice, self.data['posy'].iloc[:i+1])
        self.pos_lines['posz'].set_data(time_slice, self.data['posz'].iloc[:i+1])

        # Update accuracy plot
        self.acc_lines['orix'].set_data(time_slice, self.data['orix'].iloc[:i+1])
        self.acc_lines['oriy'].set_data(time_slice, self.data['oriy'].iloc[:i+1])
        self.acc_lines['oriz'].set_data(time_slice, self.data['oriz'].iloc[:i+1])
        self.acc_lines['oriw'].set_data(time_slice, self.data['oriw'].iloc[:i+1])

        # Update current data points
        self.total_text.set_text(f'Current time point: {i+1} ')

        return list(self.pos_lines.values()) + list(self.acc_lines.values())


    def show(self):
        """Show animation"""
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
        print("Usage: python csv准确率绘图.py <csv_file_path> [animation_interval_ms]")
        print("Example: python csv准确率绘图.py AttackDetection_000.csv 100")
        sys.exit(1)

    csv_file = sys.argv[1]
    interval = int(sys.argv[2]) if len(sys.argv) > 2 else 100

    visualizer = CSVAccuracyVisualizer(csv_file, interval=interval)
    visualizer.show()

if __name__ == "__main__":
    main()