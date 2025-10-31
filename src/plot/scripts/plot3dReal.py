#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np

class RealTime3DPlotter:
    def __init__(self, max_data_points=500):
        # ROS初始化
        rospy.init_node('realtime_3d_plotter', anonymous=True)
        
        # 数据存储
        self.max_data_points = max_data_points
        self.x_data = []
        self.y_data = [] 
        self.z_data = []
        
        # 线程锁，确保数据安全
        self.lock = threading.Lock()
        
        # 创建3D图形
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 初始化3D轨迹线 - 使用空numpy数组
        # 单元素元组解包需加,否则会被识别为括号
        self.trajectory_line, = self.ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7, label='Trajectory')
        self.current_point, = self.ax.plot([], [], [], 'ro', markersize=8, label='Current Position')
        
        # 3D图形设置
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('MAVROS 3D Real-time Trajectory')
        self.ax.legend()
        self.ax.grid(True, alpha=0.1)
        
        # 设置初始视角
        self.ax.view_init(elev=30, azim=45)
        
        # 初始化坐标轴范围
        self.x_min, self.x_max = -5, 5
        self.y_min, self.y_max = -5, 5
        self.z_min, self.z_max = 0, 10
        
        # 设置初始坐标轴范围
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_zlim(self.z_min, self.z_max)
        
        # 订阅ROS话题
        self.subscriber = rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.pose_callback)
        
        # 创建动画更新
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        
        rospy.loginfo("3D Real-time Trajectory Plotter Started")
    
    def pose_callback(self, msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Add new data
            self.x_data.append(x)
            self.y_data.append(y)
            self.z_data.append(z)
            
            # Keep data length within maximum
            if len(self.x_data) > self.max_data_points:
                self.x_data.pop(0)
                self.y_data.pop(0)
                self.z_data.pop(0)
            
            # # Print latest data every 30 data points
            # if len(self.x_data) % 30 == 0:
            #     rospy.loginfo(f"Current Position - X: {x:.2f}m, Y: {y:.2f}m, Z: {z:.2f}m")
    
    def update_plot(self, frame):
        """Update 3D plot"""
        with self.lock:
            if len(self.x_data) > 0:
                # Convert to numpy arrays to avoid shape errors
                x_array = np.array(self.x_data)
                y_array = np.array(self.y_data)
                z_array = np.array(self.z_data)
                
                # Update trajectory line - 确保使用numpy数组
                self.trajectory_line.set_data(x_array, y_array)
                self.trajectory_line.set_3d_properties(z_array)
                
                # Update current position point - 使用numpy数组而不是列表
                current_x = np.array([self.x_data[-1]])
                current_y = np.array([self.y_data[-1]])
                current_z = np.array([self.z_data[-1]])
                
                self.current_point.set_data(current_x, current_y)
                self.current_point.set_3d_properties(current_z)
                
                # Dynamically adjust axis ranges
                if len(self.x_data) > 1:
                    margin = 2.0
                    self.x_min = min(self.x_data) - margin
                    self.x_max = max(self.x_data) + margin
                    self.y_min = min(self.y_data) - margin
                    self.y_max = max(self.y_data) + margin
                    self.z_min = max(0, min(self.z_data) - margin)  # Z-axis minimum is 0
                    self.z_max = max(self.z_data) + margin
                    
                    # Set axis ranges
                    self.ax.set_xlim(self.x_min, self.x_max)
                    self.ax.set_ylim(self.y_min, self.y_max)
                    self.ax.set_zlim(self.z_min, self.z_max)
    
    def run(self):
        """Run the plot"""
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            rospy.loginfo("Plot interrupted by user")
        finally:
            self.subscriber.unregister()
            rospy.loginfo("3D Real-time Trajectory Plotter Closed")

if __name__ == '__main__':
    try:
        # Get configuration from parameter server
        max_points = rospy.get_param('~max_data_points', 500)
        
        # Create plotter and run
        plotter = RealTime3DPlotter(max_points)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
