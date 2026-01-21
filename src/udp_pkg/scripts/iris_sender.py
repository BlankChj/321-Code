import socket
import sys
import struct
import rospy
from message_filters import Subscriber
from geometry_msgs.msg import PoseStamped

server_ip = None
port_id = None

def udp_client(server_host_list, message, server_port=9200):
    for server_host in server_host_list:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
        s.setblocking(False)
        s.sendto(message, (server_host, server_port))


def callback(msg):
    global server_ip, port_id
    floats = [msg.pose.position.x, msg.pose.position.y, msg.pose.position.z]
    message = struct.pack('3d', *floats)
    udp_client(server_ip, message, (9200 + port_id))


if __name__ == "__main__":
    server_ip = sys.argv[1]
    port_id = int(sys.argv[2])
    rospy.init_node(f"iris_sender_node")
    pose_sub = Subscriber('/mavros/local_position/pose', PoseStamped, callback, queue_size=10)
    while True:
        try:
            pass
        except KeyboardInterrupt:
            sys.exit()
