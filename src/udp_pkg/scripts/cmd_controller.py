#!/usr/bin/env python3

import socket
import struct
import time
import sys
import select
import tty
import termios
import threading
import signal

running = True
settings = None

def udp_client(server_host, message, server_port=8200):
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 8192)
            s.setblocking(False)
            s.sendto(message, (server_host, server_port))
    except Exception as e:
        print(f"发送数据出错: {e}", file=sys.stderr)

def getKey():
    global key, running
    key = ''
    while running:
        try:
            tty.setraw(sys.stdin.fileno())
            rlist, _, _ = select.select([sys.stdin], [], [], 0.05)
            if rlist:
                key = sys.stdin.read(1)
                if ord(key) == 3:
                    running = False
                    break
            else:
                key = ''
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        except Exception as e:
            if running:
                print(f"键盘输入错误: {e}", file=sys.stderr)
            break

def signal_handler(signum, frame):
    global running
    running = False

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    
    server_ip = sys.argv[2:] if len(sys.argv) > 2 else []
    if not server_ip:
        print("请提供至少一个服务器IP地址作为参数", file=sys.stderr)
        sys.exit(1)
    
    settings = termios.tcgetattr(sys.stdin)
    sender_thread = threading.Thread(target=getKey, daemon=True)
    sender_thread.start()
    
    mode = 0
    pos_id = 0
    pos_num = int(sys.argv[1])
    
    try:
        while running:
            if key == '0':
                mode = 0
            elif key == '1':
                mode = 1
            elif key == '2':
                mode = 2
            if mode == 1:
                pos_id = (pos_id + 1) % pos_num
            print(f"mode = {mode}, pos_id = {pos_id}")
            message = struct.pack('2i', mode, pos_id)
            for ip in server_ip:
                udp_client(ip, message)
            time.sleep(0.05)

    finally:
        message = struct.pack('2i', 2, pos_id)
        for ip in server_ip:
            udp_client(ip, message)
            
        running = False
        sender_thread.join(timeout=0.5)
        
        if settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        
        print("\n程序已成功退出")
        sys.exit(0)
