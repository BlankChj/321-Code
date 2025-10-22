#!/usr/bin/env python3

import socket
import threading
import time
import sys
import struct
import select
import rospy
import numpy as np
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, TwistStamped
from udp_pkg.msg import PositionVelocityAccel
from collections import defaultdict, deque
from queue import Queue
from threading import Lock
import os
import copy
import psutil

received_data = defaultdict(deque)
send_queue = Queue(maxsize=10)
port_locks = defaultdict(Lock)
attack_mode = 0.0
attack_power = 0.0

def set_thread_priority(thread, priority=80):
    thread_id = thread.native_id
    os.system(f"chrt -f -p {priority} {thread_id}")

class MultiPortListener(threading.Thread):
    def __init__(self, ports, ip, buffer_size=112):
        super().__init__()
        self.ports = ports
        self.buffer_size = buffer_size
        self.running = False
        self.sockets = []
        self.ip = ip

    def run(self):
        self.running = True
        for port in self.ports:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            sock.setblocking(False)
            sock.bind((self.ip, port))
            self.sockets.append(sock)
        print(f"单线程监听端口: {self.ports}")

        while self.running:
            try:
                readable, _, _ = select.select(self.sockets, [], [], 0.01)
                for sock in readable:
                    data, addr = sock.recvfrom(self.buffer_size)
                    port = sock.getsockname()[1]
                    self.handle_data(data, addr, port)
            except Exception as e:
                time.sleep(0.001)

    def handle_data(self, data, client_address, port):
        try:
            receive_time = time.time()
            with port_locks[port]:
                received_data[port].append((client_address, data, receive_time))
                if len(received_data[port]) > 10:
                    received_data[port].popleft()
        except Exception as e:
            print(f"处理数据出错: {e}")

    def stop(self):
        self.running = False
        for sock in self.sockets:
            try:
                sock.shutdown(socket.SHUT_RDWR)
                sock.close()
            except Exception as e:
                print(f"关闭socket错误: {e}")
            sock = None


def get_latest_data(port=None):
    if port is not None:
        with port_locks[port]:
            return received_data.get(port, [])
    else:
        latest_data = {}
        for p in received_data:
            with port_locks[p]:
                if received_data[p]:
                    latest_data[p] = received_data[p][-1]
        return latest_data


class UDPSender(threading.Thread):
    def __init__(self, ip, sender_port=None, encoding='utf-8'):
        super().__init__()
        self.encoding = encoding
        self.running = False
        self.socket = None
        self.sender_port = sender_port
        self.ip = ip

    def run(self):
        self.running = True
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
            self.socket.setblocking(False)
            if self.sender_port:
                self.socket.bind((self.ip, self.sender_port))
                print(f"UDP发送线程已启动，使用端口 {self.sender_port} 发送消息...")
            else:
                print("UDP发送线程已启动，使用系统自动分配的端口发送消息...")

            while self.running:
                try:
                    target_ip, target_port, message = send_queue.get(timeout=1.0)
                    self.socket.sendto(message, (target_ip, target_port))
                    send_queue.task_done()
                except Exception as e:
                    if not isinstance(e, Exception) or not str(e).startswith("timed out"):
                        print(f"发送UDP消息时出错: {str(e)}")
        except Exception as e:
            print(f"UDP发送线程启动失败: {str(e)}")
        finally:
            self.close()
            print("UDP发送线程已停止")

    def stop(self):
        self.running = False

    def close(self):
        if self.socket:
            try:
                self.socket.shutdown(socket.SHUT_RDWR)
                self.socket.close()
            except Exception as e:
                print(f"关闭发送socket时出错: {str(e)}")
            self.socket = None


def send_udp_message(target_ip, target_port, message):
    if not send_queue.full():
        send_queue.put((target_ip, target_port, message), block=False)
    else:
        with send_queue.mutex:
            send_queue.queue.popleft()
            send_queue.put((target_ip, target_port, message), block=False)


class Agent:
    def __init__(self, num, ids, server_host_list):
        rospy.init_node(f"iris_{ids}")
        self.num = num
        self.ids = ids
        self.dos_flag = True
        self.port = 8001 + ids
        self.server_host_list = server_host_list
        self.ports_to_listen = [8001 + i for i in range(self.num)]
        self.pose = PositionVelocityAccel()
        self.all_pose = [PositionVelocityAccel() for _ in range(self.num)]
        self.pose_sub = Subscriber('/mavros/local_position/pose', PoseStamped)
        self.velocity_sub = Subscriber('/mavros/local_position/velocity_local', TwistStamped)
        self.imu_sub = Subscriber('/mavros/imu/data', Imu)
        self.ats = ApproximateTimeSynchronizer([self.pose_sub, self.velocity_sub, self.imu_sub], queue_size=10, slop=0.1)
        self.ats.registerCallback(self.three_topic_callback)
        self.cmd_pub = rospy.Publisher('/leader/information', PositionVelocityAccel, queue_size=10)
        self.cmd = PositionVelocityAccel()
        self.history_pose = [deque(maxlen=200) for _ in range(self.num)]

    def pose_callback(self, msg):
        self.pose = msg
    
    def three_topic_callback(self, pose_msg, velocity_msg, imu_msg):
        self.pose.frame_id = pose_msg.header.frame_id
        self.pose.stamp = pose_msg.header.stamp.to_sec()
        self.pose.x_pos = pose_msg.pose.position.x
        self.pose.y_pos = pose_msg.pose.position.y
        self.pose.z_pos = pose_msg.pose.position.z
        self.pose.x_ori = pose_msg.pose.orientation.x
        self.pose.y_ori = pose_msg.pose.orientation.y
        self.pose.z_ori = pose_msg.pose.orientation.z
        self.pose.w_ori = pose_msg.pose.orientation.w
        self.pose.x_vel = velocity_msg.twist.linear.x
        self.pose.y_vel = velocity_msg.twist.linear.y
        self.pose.z_vel = velocity_msg.twist.linear.z
        self.pose.x_acc = imu_msg.linear_acceleration.x
        self.pose.y_acc = imu_msg.linear_acceleration.y
        self.pose.z_acc = imu_msg.linear_acceleration.z

    def main(self):
        global attack_mode, attack_power
        listener = MultiPortListener(self.ports_to_listen, self.server_host_list[0])
        listener.setDaemon(True)
        listener.start()
        set_thread_priority(listener)
        listeners = [listener]
        sender = UDPSender(self.server_host_list[0], sender_port=9000)
        sender.setDaemon(True)
        sender.start()
        set_thread_priority(sender)
        time.sleep(0.1)
        print(f"已启动了 {len(listeners)} 个UDP监听线程，分别监听端口: {self.ports_to_listen}")
        print("UDP发送线程已启动，可通过send_udp_message()函数发送消息")
        print("按 Ctrl+C 停止程序...")

        def sender_func():
            send_rate = rospy.Rate(60)
            while True:
                m0 = self.pose.stamp
                m1 = self.pose.x_pos
                m2 = self.pose.y_pos
                m3 = self.pose.z_pos
                m4 = self.pose.x_ori
                m5 = self.pose.y_ori
                m6 = self.pose.z_ori
                m7 = self.pose.w_ori
                m8 = self.pose.x_vel
                m9 = self.pose.y_vel
                m10 = self.pose.z_vel
                m11 = self.pose.x_acc
                m12 = self.pose.y_acc
                m13 = self.pose.z_acc
                floats = [m0, m1, m2, m3, m4, m5, m6, m7, m8, m9, m10, m11, m12, m13]
                message = struct.pack('14d', *floats)
                for server_host in self.server_host_list:
                    send_udp_message(server_host, self.port, message)
                send_rate.sleep()

        sender_thread = threading.Thread(target=sender_func, daemon=True)
        sender_thread.start()

        def listener_func():
            ports = self.ports_to_listen
            listen_rate = rospy.Rate(60)
            while True:
                messages = get_latest_data()
                for i in range(self.num):
                    p = ports[i]
                    if p in messages:
                        with port_locks[i]:
                            message = messages[p]
                            value = struct.unpack('14d', message[1])
                            self.all_pose[i].frame_id = "world"
                            self.all_pose[i].stamp = value[0]
                            self.all_pose[i].x_pos = value[1]
                            self.all_pose[i].y_pos = value[2]
                            self.all_pose[i].z_pos = value[3]
                            self.all_pose[i].x_ori = value[4]
                            self.all_pose[i].y_ori = value[5]
                            self.all_pose[i].z_ori = value[6]
                            self.all_pose[i].w_ori = value[7]
                            self.all_pose[i].x_vel = value[8]
                            self.all_pose[i].y_vel = value[9]
                            self.all_pose[i].z_vel = value[10]
                            self.all_pose[i].x_acc = value[11]
                            self.all_pose[i].y_acc = value[12]
                            self.all_pose[i].z_acc = value[13]
                listen_rate.sleep()

        listener_thread = threading.Thread(target=listener_func, daemon=True)
        listener_thread.start()

        def attack_func(host='0.0.0.0', port=8100):
            global attack_mode, attack_power
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            s.setblocking(False)
            s.bind((host, port))
            while True:
                readable, _, _ = select.select([s], [], [], 0.01)
                if readable:
                    data, addr = s.recvfrom(16)
                    value = struct.unpack('2f', data)
                    with port_locks[port]:
                        attack_mode, attack_power = value
        
        attack_thread = threading.Thread(target=attack_func, args=(self.server_host_list[0], 8100), daemon=True)
        attack_thread.start()

        try:
            rate = rospy.Rate(60)
            while not rospy.is_shutdown():
                try:
                    with port_locks[8100]:
                        with port_locks[0]:
                            if attack_mode == 0.0:
                                self.dos_flag = True
                                self.cmd = self.all_pose[0]
                                self.history_pose[0].append(copy.deepcopy(self.cmd))
                                self.cmd.frame_id = "world_0"
                                self.cmd_pub.publish(self.cmd)
                            elif attack_mode == 1.0:
                                if self.dos_flag:
                                    self.cmd = self.all_pose[0]
                                    self.dos_flag = False
                                else:
                                    self.cmd.stamp = self.all_pose[0].stamp
                                self.history_pose[0].append(copy.deepcopy(self.cmd))
                                self.cmd.frame_id = "world_1"
                                self.cmd_pub.publish(self.cmd)
                            elif attack_mode == 2.0:
                                self.dos_flag = True
                                self.cmd = self.all_pose[0]
                                x_limit = abs(self.cmd.x_pos * attack_power)
                                y_limit = abs(self.cmd.y_pos * attack_power)
                                z_limit = abs(self.cmd.z_pos * attack_power)
                                self.cmd.x_pos = self.cmd.x_pos + np.random.uniform(-x_limit, x_limit)
                                self.cmd.y_pos = self.cmd.y_pos + np.random.uniform(-y_limit, y_limit)
                                self.cmd.z_pos = self.cmd.z_pos + np.random.uniform(-z_limit, z_limit)
                                self.history_pose[0].append(copy.deepcopy(self.cmd))
                                self.cmd.frame_id = "world_2"
                                self.cmd_pub.publish(self.cmd)
                            elif attack_mode == 3.0:
                                self.dos_flag = True
                                if len(self.history_pose[0]) > 0:
                                    self.cmd = self.history_pose[0].popleft()
                                else:
                                    self.cmd = self.all_pose[0]
                                self.history_pose[0].append(copy.deepcopy(self.all_pose[0]))
                                self.cmd.frame_id = "world_3"
                                self.cmd_pub.publish(self.cmd)
                except KeyboardInterrupt:
                    print("\n收到停止信号，正在关闭所有线程...")
                    sender.stop()
                    sender.join()
                    for listener in listeners:
                        listener.stop()
                    for listener in listeners:
                        listener.join()
                    print("所有线程已关闭，程序退出")
                try:
                    rate.sleep()
                except:
                    continue
        except KeyboardInterrupt:
            print("\n收到停止信号，正在关闭所有线程...")
            sender.stop()
            sender.join()
            for listener in listeners:
                listener.stop()
            for listener in listeners:
                listener.join()
            print("所有线程已关闭，程序退出")


if __name__ == "__main__":
    num_args = len(sys.argv)
    num = num_args - 2
    ids = int(sys.argv[1])
    server_host_list = []
    for i in range(2, num_args):
        server_host_list.append(sys.argv[i])
    iris = Agent(num, ids, server_host_list)
    time.sleep(1)
    iris.main()
