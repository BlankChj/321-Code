#!/usr/bin/env python3
import sys
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np



class TrajectoryVisualizer:
    def __init__(self, csv_files, interval=200):

        self.csv_files = csv_files
        self.interval = interval
        self.trajectories = []
        self.n_frames = 0
        self.load_data()

    def load_data(self):
        """Load all trajectory data"""
        for idx, filename in enumerate(self.csv_files):
            traj = self.read_trajectory(filename)
            if traj.size == 0:
                print(f"Warning: File {filename} has no valid trajectory data, skipped.")
                continue
            self.trajectories.append(traj)
            print(f"Successfully loaded trajectory {idx}: {filename}, points: {len(traj)}")
        
        if not self.trajectories:
            print("Error: No valid trajectory data")
            sys.exit(1)
        
        self.n_frames = max([len(d) for d in self.trajectories])
        print(f"Total trajectories: {len(self.trajectories)}, Max frames: {self.n_frames}")

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
        self.fig.suptitle('IRIS 3D Trajectory', fontsize=16, fontweight='bold')

        cmap = plt.get_cmap('tab20')

        # Create line and scatter objects
        self.lines = []
        self.scatters_start = []
        self.scatters_end = []

        for idx, traj in enumerate(self.trajectories):
            color = cmap(idx % 20)
            line = self.ax.plot([], [], [], color=color, linewidth=2, linestyle='--', label=f'iris{idx}')[0]
            scatter_start = self.ax.scatter([], [], [], color=color, s=50, marker='o', edgecolors='k', zorder=5)
            scatter_end = self.ax.scatter([], [], [], color=color, s=50, marker='s', edgecolors='k', zorder=5)
            
            self.lines.append(line)
            self.scatters_start.append(scatter_start)
            self.scatters_end.append(scatter_end)

        # Set axis labels
        self.ax.set_xlabel('X', fontsize=12, fontweight='bold')
        self.ax.set_ylabel('Y', fontsize=12, fontweight='bold')
        self.ax.set_zlabel('Z', fontsize=12, fontweight='bold')
        self.ax.grid(True, alpha=0.4, linestyle='--', linewidth=0.8)
        self.ax.legend(title="UAV", loc='best', fontsize=10, framealpha=0.9, fancybox=True, shadow=True)

        # Set axis range (avoid animation jitter)
        all_x = np.concatenate([traj[:, 0] for traj in self.trajectories])
        all_y = np.concatenate([traj[:, 1] for traj in self.trajectories])
        all_z = np.concatenate([traj[:, 2] for traj in self.trajectories])
        
        self.ax.set_xlim(all_x.min(), all_x.max())
        self.ax.set_ylim(all_y.min(), all_y.max())
        self.ax.set_zlim(all_z.min(), all_z.max())

        # Total data points text
        self.total_text = self.fig.text(0.5, 0.90, f'Trajectories: {len(self.trajectories)}', 
                                        ha='center', fontsize=12, fontweight='bold')

    def animate(self, i):
        """Animation update function"""
        for line, traj, scatter_start, scatter_end in zip(self.lines, self.trajectories, 
                                                           self.scatters_start, self.scatters_end):
            # Update trajectory line
            line.set_data(traj[:i, 0], traj[:i, 1])
            line.set_3d_properties(traj[:i, 2])
            
            # Update start marker
            if i > 0:
                scatter_start._offsets3d = ([traj[0, 0]], [traj[0, 1]], [traj[0, 2]])
            
            # Update end marker
            if i < traj.shape[0]:
                scatter_end._offsets3d = ([traj[i-1, 0]], [traj[i-1, 1]], [traj[i-1, 2]])
            else:
                scatter_end._offsets3d = ([traj[-1, 0]], [traj[-1, 1]], [traj[-1, 2]])
        
        return self.lines + self.scatters_start + self.scatters_end

    def show(self):
        """Show animation"""
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
        print("Usage: python 3d轨迹.py <csv_file1> <csv_file2> ... [animation_interval_ms]")
        print("Example: python 3d轨迹.py iris_0_data.csv iris_1_data.csv 200")
        sys.exit(1)

    csv_files = sys.argv[1:-1] if len(sys.argv) > 2 and sys.argv[-1].isdigit() else sys.argv[1:]
    interval = int(sys.argv[-1]) if len(sys.argv) > 1 and sys.argv[-1].isdigit() else 200

    visualizer = TrajectoryVisualizer(csv_files, interval=interval)
    visualizer.show()

if __name__ == "__main__":
    main()