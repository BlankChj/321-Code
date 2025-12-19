#!/usr/bin/env python
import rospy
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from geometry_msgs.msg import PoseStamped
from udp_pkg.msg import PositionVelocityAccel
from kalman_filter.msg import Vector3Stamped
import threading
import numpy as np
import message_filters


class PosEstPlotter:
    def __init__(self, max_data_points=4000):
        # ROS初始化
        rospy.init_node('PosEstPlotter', anonymous=True)
        
        # 数据存储
        self.max_data_points = max_data_points

        self.t_data = []
        self.x_data = []
        self.y_data = [] 
        self.z_data = []

        self.t_attack_data = []
        self.x_attack_data = []
        self.y_attack_data = [] 
        self.z_attack_data = []

        self.t_rkf_data = []
        self.x_rkf_data = []
        self.y_rkf_data = [] 
        self.z_rkf_data = []

        self.t_kf_data = []
        self.x_kf_data = []
        self.y_kf_data = [] 
        self.z_kf_data = []

        # 线程锁，确保数据安全
        self.lock = threading.Lock()
        
        # 创建3D图形
        # self.fig = plt.figure(figsize=(10, 8))
        # self.ax = self.fig.add_subplot(111, projection='3d')
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(10, 8))
        # 初始化3D轨迹线 - 使用空numpy数组
        # 单元素元组解包需加,否则会被识别为括号
        self.x_line_normal, = self.ax1.plot([], [], 'r-', linewidth=2, alpha=0.7, label=' Normal Data')
        self.x_line_attack, = self.ax1.plot([], [], 'b-', linewidth=2, alpha=0.7, label=' Attack Data')
        self.x_line_kf, = self.ax1.plot([], [], 'g-', linewidth=2, alpha=0.7, label=' KF Data')
        self.x_line_rkf, = self.ax1.plot([], [], 'y-', linewidth=2, alpha=0.7, label=' RKF Data')

        self.y_line_normal, = self.ax2.plot([], [], 'r-', linewidth=2, alpha=0.7, label=' Normal Data')
        self.y_line_attack, = self.ax2.plot([], [], 'b-', linewidth=2, alpha=0.7, label=' Attack Data')
        self.y_line_kf, = self.ax2.plot([], [], 'g-', linewidth=2, alpha=0.7, label=' KF Data')
        self.y_line_rkf, = self.ax2.plot([], [], 'y-', linewidth=2, alpha=0.7, label=' RKF Data')

        self.z_line_normal, = self.ax3.plot([], [], 'r-', linewidth=2, alpha=0.7, label=' Normal Data')
        self.z_line_attack, = self.ax3.plot([], [], 'b-', linewidth=2, alpha=0.7, label=' Attack Data')
        self.z_line_kf, = self.ax3.plot([], [], 'g-', linewidth=2, alpha=0.7, label=' KF Data')
        self.z_line_rkf, = self.ax3.plot([], [], 'y-', linewidth=2, alpha=0.7, label=' RKF Data')

        # 2D图形设置
        self.ax1.set_xlabel('t (s)')
        self.ax1.set_ylabel('X (m)')
        self.ax1.set_title('Real-time X Trajectory')
        self.ax1.legend()
        self.ax1.grid(True, alpha=0.1)
        
        self.ax2.set_xlabel('t (s)')
        self.ax2.set_ylabel('Y (m)')
        self.ax2.set_title('Real-time Y Trajectory')
        self.ax2.legend()
        self.ax2.grid(True, alpha=0.1)

        self.ax3.set_xlabel('t (s)')
        self.ax3.set_ylabel('Z (m)')
        self.ax3.set_title('Real-time Z Trajectory')
        self.ax3.legend()
        self.ax3.grid(True, alpha=0.1)
        
        
        # 初始化坐标轴范围
        self.x_min, self.x_max = 0, 5
        self.y_min, self.y_max = -2, 2
        self.z_min, self.z_max = -2, 2
        
        # 设置初始坐标轴范围
        self.ax1.set_xlim(self.x_min, self.x_max)
        self.ax1.set_ylim(self.y_min, self.y_max)

        self.ax2.set_xlim(self.x_min, self.x_max)
        self.ax2.set_ylim(self.y_min, self.y_max)
        
        self.ax3.set_xlim(self.x_min, self.x_max)
        self.ax3.set_ylim(self.y_min, self.y_max)
        
        # 订阅ROS话题
        self.subscriber1 = rospy.Subscriber('/vrpn_client_node/iris6/pose', PoseStamped, self.pose_callback)
        self.subscriber2 = rospy.Subscriber('/leader/information', PositionVelocityAccel, self.pose_callback_attack)
        self.subscriber3 = rospy.Subscriber('/leader/kf/pos', Vector3Stamped, self.pose_callback_kf)
        self.subscriber4 = rospy.Subscriber('/leader/rkf/pos', Vector3Stamped, self.pose_callback_rkf)


        # 创建动画更新
        self.animation = FuncAnimation(self.fig, self.update_plot, interval=50, cache_frame_data=False)
        
        rospy.loginfo("XYZ Real-time Trajectory Plotter Started")
    
    def pose_callback(self, leader_msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x1 = leader_msg.pose.position.x
            y1 = leader_msg.pose.position.y
            z1 = leader_msg.pose.position.z
            t1 = rospy.Time.now().to_sec()
            # Add new data
            self.t_data.append(t1)
            self.x_data.append(x1)
            self.y_data.append(y1)
            self.z_data.append(z1)

            # Keep data length within maximum
            if len(self.x_data) > self.max_data_points:
                self.x_data.pop(0)
                self.y_data.pop(0)
                self.z_data.pop(0)

    def pose_callback_attack(self, leader_msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x1 = leader_msg.x_pos
            y1 = leader_msg.y_pos
            z1 = leader_msg.z_pos
            t1 = rospy.Time.now().to_sec()
            # Add new data
            self.t_attack_data.append(t1)
            self.x_attack_data.append(x1)
            self.y_attack_data.append(y1)
            self.z_attack_data.append(z1)

            # Keep data length within maximum
            if len(self.x_attack_data) > self.max_data_points:
                self.x_attack_data.pop(0)
                self.y_attack_data.pop(0)
                self.z_attack_data.pop(0)
    def pose_callback_kf(self, leader_msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x1 = leader_msg.x
            y1 = leader_msg.y
            z1 = leader_msg.z
            t1 = rospy.Time.now().to_sec()
            # Add new data
            self.t_kf_data.append(t1)
            self.x_kf_data.append(x1)
            self.y_kf_data.append(y1)
            self.z_kf_data.append(z1)

            # Keep data length within maximum
            if len(self.x_kf_data) > self.max_data_points:
                self.x_kf_data.pop(0)
                self.y_kf_data.pop(0)
                self.z_kf_data.pop(0)
    def pose_callback_rkf(self, leader_msg):
        """PoseStamped message callback - extract only XYZ coordinates"""
        with self.lock:
            # Extract position data
            x1 = leader_msg.x
            y1 = leader_msg.y
            z1 = leader_msg.z
            t1 = rospy.Time.now().to_sec()
            # Add new data
            self.t_rkf_data.append(t1)
            self.x_rkf_data.append(x1)
            self.y_rkf_data.append(y1)
            self.z_rkf_data.append(z1)

            # Keep data length within maximum
            if len(self.x_rkf_data) > self.max_data_points:
                self.x_rkf_data.pop(0)
                self.y_rkf_data.pop(0)
                self.z_rkf_data.pop(0)

    def update_plot(self, frame):
        """Update 3D plot"""
        with self.lock:
            if len(self.x_data) > 0 and len(self.x_attack_data) > 0 and len(self.x_kf_data) > 0 and len(self.x_rkf_data) > 0:
                
                # Convert to numpy arrays to avoid shape errors
                x_array = np.array(self.x_data)
                y_array = np.array(self.y_data)
                z_array = np.array(self.z_data)
                t_array = np.array(self.t_data)

                self.x_line_normal.set_data(t_array, x_array)
                self.y_line_normal.set_data(t_array, y_array)
                self.z_line_normal.set_data(t_array, z_array)

                x_attack_array = np.array(self.x_attack_data)
                y_attack_array = np.array(self.y_attack_data)
                z_attack_array = np.array(self.z_attack_data)
                t_attack_array = np.array(self.t_attack_data)

                self.x_line_attack.set_data(t_attack_array, x_attack_array)
                self.y_line_attack.set_data(t_attack_array, y_attack_array)
                self.z_line_attack.set_data(t_attack_array, z_attack_array)

                x_kf_array = np.array(self.x_kf_data)
                y_kf_array = np.array(self.y_kf_data)
                z_kf_array = np.array(self.z_kf_data)
                t_kf_array = np.array(self.t_kf_data)

                self.x_line_kf.set_data(t_kf_array, x_kf_array)
                self.y_line_kf.set_data(t_kf_array, y_kf_array)
                self.z_line_kf.set_data(t_kf_array, z_kf_array)

                x_rkf_array = np.array(self.x_rkf_data)
                y_rkf_array = np.array(self.y_rkf_data)
                z_rkf_array = np.array(self.z_rkf_data)
                t_rkf_array = np.array(self.t_rkf_data)

                self.x_line_rkf.set_data(t_rkf_array, x_rkf_array)
                self.y_line_rkf.set_data(t_rkf_array, y_rkf_array)
                self.z_line_rkf.set_data(t_rkf_array, z_rkf_array)
                # Dynamically adjust axis ranges
                if len(self.x_data) > 1:
                    margin = 2.0
                    self.t_min = min(self.t_data) - margin
                    self.t_max = max(self.t_data) + margin

                    self.x_min = min(self.x_data) - margin
                    self.x_max = max(self.x_data) + margin

                    self.y_min = min(self.y_data) - margin
                    self.y_max = max(self.y_data) + margin

                    self.z_min = min(self.z_data) - margin
                    self.z_max = max(self.z_data) + margin

                    # Set axis ranges
                    self.ax1.set_xlim(self.t_min, self.t_max)
                    self.ax2.set_xlim(self.t_min, self.t_max)
                    self.ax3.set_xlim(self.t_min, self.t_max)

                    self.ax1.set_ylim(self.x_min, self.x_max)
                    self.ax2.set_ylim(self.y_min, self.y_max)
                    self.ax3.set_ylim(self.z_min, self.z_max)


    def run(self):
        """Run the plot"""
        try:
            plt.show(block=True)
        except KeyboardInterrupt:
            rospy.loginfo("Plot interrupted by user")
        finally:
            self.subscriber1.unregister()
            self.subscriber2.unregister()
            self.subscriber3.unregister()
            self.subscriber4.unregister()

            rospy.loginfo("3D Real-time Trajectory Plotter Closed")

if __name__ == '__main__':
    try:
        # Get configuration from parameter server
        max_points = rospy.get_param('~max_data_points',2000)
        
        # Create plotter and run
        plotter = PosEstPlotter(max_points)
        plotter.run()
    except rospy.ROSInterruptException:
        pass
