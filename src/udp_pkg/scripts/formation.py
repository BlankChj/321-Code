#!/usr/bin/env python3

import socket
import struct
import select
import threading
from threading import Lock
from collections import defaultdict
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
import sys

mode, pos_id = 0, 0
locks = defaultdict(Lock)

class Formation():
    def __init__(self, pos_num, num, ids):
        rospy.init_node(f"formation_{ids}")
        self.num = num
        self.ids = ids
        self.pos_num = pos_num
        self.x_list = [(0 + np.cos(2 * i * np.pi / self.pos_num)) for i in range(self.pos_num)]
        self.y_list = [(0 + np.sin(2 * i * np.pi / self.pos_num)) for i in range(self.pos_num)]
        self.bias = self.ids * self.pos_num // self.num
        self.cmd_pose = PoseStamped()
        self.cmd_pub = rospy.Publisher(f'/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    
    def main(self):
        global mode, pos_id

        def cmd_func(host='0.0.0.0', port=8200):
            global mode, pos_id
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
            s.setblocking(False)
            s.bind((host, port))
            while True:
                readable, _, _ = select.select([s], [], [], 0.01)
                if readable:
                    data, addr = s.recvfrom(16)
                    value = struct.unpack('2i', data)
                    with locks[0]:
                        mode, pos_id = value
        
        cmd_thread = threading.Thread(target=cmd_func, daemon=True)
        cmd_thread.start()

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            try:
                with locks[0]:
                    ii = (self.bias + pos_id) % self.pos_num
                    self.cmd_pose.pose.position.x = self.x_list[ii]
                    self.cmd_pose.pose.position.y = self.y_list[ii]
                    if mode == 0 or mode == 1:
                        self.cmd_pose.pose.position.z = 0.8
                    elif mode == 2:
                        self.cmd_pose.pose.position.z = 0.05
                    self.cmd_pub.publish(self.cmd_pose)
            except KeyboardInterrupt:
                break
            try:
                rate.sleep()
            except:
                continue

if __name__ == "__main__":
    pos_num = int(sys.argv[1])
    num = int(sys.argv[2])
    ids = int(sys.argv[3])
    formation = Formation(pos_num, num, ids)
    formation.main()
