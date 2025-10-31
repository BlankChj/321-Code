import socket
import struct
import numpy as np
import sys

def udp_client(server_host, message, server_port=8100):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
    s.setblocking(False)
    s.sendto(message, (server_host, server_port))


if __name__ == "__main__":
    server_ip = input("请输入攻击的IP地址: ")
    while True:
        try:
            print("0:stop attack, 1:DoS attack, 2:FDI attack, 3:replay attack")
            attack_mode = float(input("请输入攻击方式: "))
            floats = []
            floats.append(attack_mode)
            if attack_mode == 0.0 or attack_mode == 3.0:
                attack_power = 0.0
                floats.append(attack_power)
            elif attack_mode == 1.0:
                attack_power = float(int(input("请输入DoS攻击的频率(20.0 ~ 300.0): ")))
                attack_power = np.clip(attack_power, 2.0, 300.0)
                floats.append(attack_power)
            elif attack_mode == 2.0:
                attack_power = float(input("请输入FDI攻击的强度(0.0 ~ 1.0): "))
                attack_power = np.clip(attack_power, 0.0, 1.0)
                floats.append(attack_power)
            print()
            message = struct.pack('2f', *floats)
            udp_client(server_ip, message)
        except KeyboardInterrupt:
            attack_mode = 0.0
            attack_power = 0.0
            message = struct.pack('2f', *[attack_mode, attack_power])
            udp_client(server_ip, message)
            print()
            print("程序退出！")
            sys.exit()
