#!/usr/bin/env python3

import socket
import threading
import time
import sys
import struct
import select
import rospy
import numpy as np
import xgboost as xgb
import pandas as pd
from geometry_msgs.msg import PoseStamped
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
MODEL_PATH = "xgboost_attack_model.json"

def set_thread_priority(thread, priority=95):
    thread_id = thread.native_id
    os.system(f"chrt -f -p {priority} {thread_id}")

class XGBoostAttackDetector:
    def __init__(self, model_path: str):
        self.FEATURE_COLS = ["pos_x", "pos_y", "pos_z", "ori_x", "ori_y", "ori_z", "ori_w", "secs_relative"]
        self.first_secs = None
        self.model_path = model_path
        self.load_model()
    
    def load_model(self):
        if os.path.exists(self.model_path):
            self.model = xgb.XGBClassifier(n_jobs=1)
            self.model.load_model(self.model_path)
        else:
            raise FileNotFoundError(f"模型文件未找到：{self.model_path}")
    
    def predict_dataframe(self, features_df: pd.DataFrame) -> tuple:
        pred_label = self.model.predict(features_df)
        pred_proba = self.model.predict_proba(features_df)
        confidence = pred_proba.max(axis=1)
        return pred_label[0], confidence[0]
    
    def predict(self, pose: PoseStamped) -> tuple:
        if self.first_secs is None:
            self.first_secs = pose.header.stamp.secs
        px = pose.pose.position.x
        py = pose.pose.position.y
        pz = pose.pose.position.z
        ox = pose.pose.orientation.x
        oy = pose.pose.orientation.y
        oz = pose.pose.orientation.z
        ow = pose.pose.orientation.w
        secs_relative = pose.header.stamp.secs - self.first_secs
        feature_rows = [[px, py, pz, ox, oy, oz, ow, secs_relative]]
        data = pd.DataFrame(feature_rows, columns=self.FEATURE_COLS)
        return self.predict_dataframe(data)

class MultiPortListener(threading.Thread):
    def __init__(self, ports, ip, buffer_size=28):
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
        self.detector = XGBoostAttackDetector(MODEL_PATH)
        self.num = num
        self.ids = ids
        self.port = 8001 + ids
        self.server_host_list = server_host_list
        self.ports_to_listen = [8001 + i for i in range(self.num)]
        self.pose = PoseStamped()
        self.all_pose = [PoseStamped() for _ in range(self.num)]
        self.pose_sub = rospy.Subscriber('/vrpn_client_node/iris4/pose', PoseStamped, self.pose_callback, queue_size=10)
        self.pose.pose.position.x = 6.66
        self.pose.pose.position.y = 6.66
        self.pose.pose.position.z = 6.66
        self.pose.pose.orientation.x = 0.00
        self.pose.pose.orientation.y = 0.00
        self.pose.pose.orientation.z = 0.00
        self.pose.pose.orientation.w = 0.00
        self.cmd_pub = rospy.Publisher('/leader/pose', PoseStamped, queue_size=10)
        self.cmd = PoseStamped()
        self.history_pose = [deque(maxlen=200) for _ in range(self.num)]

    def pose_callback(self, msg):
        self.pose = msg

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
                m1 = self.pose.pose.position.x
                m2 = self.pose.pose.position.y
                m3 = self.pose.pose.position.z
                m4 = self.pose.pose.orientation.x
                m5 = self.pose.pose.orientation.y
                m6 = self.pose.pose.orientation.z
                m7 = self.pose.pose.orientation.w
                floats = [m1, m2, m3, m4, m5, m6, m7]
                message = struct.pack('7f', *floats)
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
                        message = messages[p]
                        value = struct.unpack('7f', message[1])
                        self.all_pose[i].header.stamp = rospy.Time.now()
                        self.all_pose[i].header.frame_id = "world"
                        self.all_pose[i].pose.position.x = value[0]
                        self.all_pose[i].pose.position.y = value[1]
                        self.all_pose[i].pose.position.z = value[2]
                        self.all_pose[i].pose.orientation.x = value[3]
                        self.all_pose[i].pose.orientation.y = value[4]
                        self.all_pose[i].pose.orientation.z = value[5]
                        self.all_pose[i].pose.orientation.w = value[6]
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
                        if attack_mode == 0.0 or attack_mode == 1.0:
                            self.cmd = self.all_pose[0]
                            self.history_pose[0].append(copy.deepcopy(self.cmd))
                            self.cmd.header.frame_id = f"world_{int(attack_mode)}"
                        elif attack_mode == 2.0:
                            self.cmd = self.all_pose[0]
                            x_limit = abs(self.cmd.pose.position.x * attack_power)
                            y_limit = abs(self.cmd.pose.position.y * attack_power)
                            z_limit = abs(self.cmd.pose.position.z * attack_power)
                            self.cmd.pose.position.x = self.cmd.pose.position.x + np.random.uniform(-x_limit, x_limit)
                            self.cmd.pose.position.y = self.cmd.pose.position.y + np.random.uniform(-y_limit, y_limit)
                            self.cmd.pose.position.z = self.cmd.pose.position.z + np.random.uniform(-z_limit, z_limit)
                            self.history_pose[0].append(copy.deepcopy(self.cmd))
                            self.cmd.header.frame_id = f"world_{int(attack_mode)}"
                        elif attack_mode == 3.0:
                            if len(self.history_pose[0]) > 0:
                                self.cmd = self.history_pose[0].popleft()
                            else:
                                self.cmd = self.all_pose[0]
                            self.history_pose[0].append(copy.deepcopy(self.all_pose[0]))
                            self.cmd.header.frame_id = f"world_{int(attack_mode)}"
                        pred_label, confidences = self.detector.predict(self.cmd)
                        print(f"predict_label:{pred_label}, true_label:{int(attack_mode)}")
                        self.cmd_pub.publish(self.cmd)
                    if attack_mode == 1.0:
                        time.sleep(1 / attack_power)
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
