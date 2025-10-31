#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np
import message_filters

class RealTime3DPlotter:
    def __init__(self, max_data_points=4000):
        # ROS初始化
        rospy.init_node('realtime_3d_plotter', anonymous=True)
        
        # 数据存储
        self.max_data_points = max_data_points
        self.x1_data = []
        self.y1_data = [] 
        self.z1_data = []
        
        self.x2_data = []
        self.y2_data = [] 
        self.z2_data = []

        # 线程锁，确保数据安全
        self.lock = threading.Lock()
        
        # 创建3D图形
        self.fig = plt.figure(figsize=(10, 8))
        self.ax = self.fig.add_subplot(111, projection='3d')
        
        # 初始化3D轨迹线 - 使用空numpy数组
        # 单元素元组解包需加,否则会被识别为括号
        self.trajectory_line1, = self.ax.plot([], [], [], 'b-', linewidth=2, alpha=0.7, label='Leader Trajectory')
        self.current_point1, = self.ax.plot([], [], [], 'ro', markersize=8, label='Leader Position')
        
        self.trajectory_line2, = self.ax.plot([], [], [], 'g-', linewidth=2, alpha=0.7, label='Follower Trajectory')
        self.current_point2, = self.ax.plot([], [], [], 'yo', markersize=8, label='Follower Position')
        # 3D图形设置
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_zlabel('Z (m)')
        self.ax.set_title('MAVROS 3D Real-time Trajectory')
        self.ax.legend()
        self.ax.grid(True, alpha=0.1)
        
        # 设置初始视角
        # self.ax.view_init(elev=30, azim=45)
        
        # 初始化坐标轴范围
        self.x_min, self.x_max = -1.5, 1.5
        self.y_min, self.y_max = -1.5, 1.5
        self.z_min, self.z_max = 0, 1
        
        # 设置初始坐标轴范围
        self.ax.set_xlim(self.x_min, self.x_max)
        self.ax.set_ylim(self.y_min, self.y_max)
        self.ax.set_zlim(self.z_min, self.z_max)
        
        # 订阅ROS话题
        self.subscriber1 = message_filters.Subscriber('/vrpn_client_node/iris3/pose', PoseStamped)

        self.subscriber2 = message_filters.Subscriber('/vrpn_client_node/iris4/pose', PoseStamped)

        # 同步订阅器
        ats = message_filters.ApproximateTimeSynchronizer(
            [self.subscriber1, self.subscriber2], 
            queue_size=10,
            slop=0.1  # 100ms 时间容差
        )
        ats.registerCallback(self.approximate_sync_callback)

        # 创建动画更新
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        
        rospy.loginfo("3D Real-time Trajectory Plotter Started")
    
    def approximate_sync_callback(self, leader_msg, follower_msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x1 = leader_msg.pose.position.x
            y1 = leader_msg.pose.position.y
            z1 = leader_msg.pose.position.z
            
            x2 = follower_msg.pose.position.x
            y2 = follower_msg.pose.position.y
            z2 = follower_msg.pose.position.z

            # Add new data
            self.x1_data.append(x1)
            self.y1_data.append(y1)
            self.z1_data.append(z1)

            self.x2_data.append(x2)
            self.y2_data.append(y2)
            self.z2_data.append(z2) 

            # Keep data length within maximum
            if len(self.x1_data) > self.max_data_points:
                self.x1_data.pop(0)
                self.y1_data.pop(0)
                self.z1_data.pop(0)
                self.x2_data.pop(0)
                self.y2_data.pop(0)
                self.z2_data.pop(0)          
            # # Print latest data every 30 data points
            # if len(self.x_data) % 30 == 0:
            #     rospy.loginfo(f"Current Position - X: {x:.2f}m, Y: {y:.2f}m, Z: {z:.2f}m")
    
    def update_plot(self, frame):
        """Update 3D plot"""
        with self.lock:
            if len(self.x1_data) > 0:
                # Convert to numpy arrays to avoid shape errors
                x1_array = np.array(self.x1_data)
                y1_array = np.array(self.y1_data)
                z1_array = np.array(self.z1_data)

                x2_array = np.array(self.x2_data)
                y2_array = np.array(self.y2_data)
                z2_array = np.array(self.z2_data)              
                # Update trajectory line - 确保使用numpy数组
                self.trajectory_line1.set_data(x1_array, y1_array)
                self.trajectory_line1.set_3d_properties(z1_array)
                
                self.trajectory_line2.set_data(x2_array, y2_array)
                self.trajectory_line2.set_3d_properties(z2_array)

                # Update current position point - 使用numpy数组而不是列表
                current_x1 = np.array([self.x1_data[-1]])
                current_y1 = np.array([self.y1_data[-1]])
                current_z1 = np.array([self.z1_data[-1]])
                
                current_x2 = np.array([self.x2_data[-1]])
                current_y2 = np.array([self.y2_data[-1]])
                current_z2 = np.array([self.z2_data[-1]])

                self.current_point1.set_data(current_x1, current_y1)
                self.current_point1.set_3d_properties(current_z1)
                
                self.current_point2.set_data(current_x2, current_y2)
                self.current_point2.set_3d_properties(current_z2)

                # Dynamically adjust axis ranges
                if len(self.x1_data) > 1:
                    # margin = 2.0
                    # self.x_min = min(self.x1_data + self.x2_data) - margin
                    # self.x_max = max(self.x1_data + self.x2_data) + margin
                    # self.y_min = min(self.y1_data + self.y2_data) - margin
                    # self.y_max = max(self.y1_data + self.y2_data) + margin
                    # self.z_min = max(0, min(self.z1_data + self.z2_data) - margin)  # Z-axis minimum is 0
                    # self.z_max = max(self.z1_data + self.z2_data) + margin
                    
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
            self.subscriber1.unregister()
            self.subscriber2.unregister()
            rospy.loginfo("3D Real-time Trajectory Plotter Closed")

if __name__ == '__main__':
    try:
        # Get configuration from parameter server
        max_points = rospy.get_param('~max_data_points',4000)
        
        # Create plotter and run
        plotter = RealTime3DPlotter(max_points)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
