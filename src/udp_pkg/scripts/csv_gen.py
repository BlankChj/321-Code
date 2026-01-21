import socket
import csv
import signal
import sys
import time
import select
import struct
from collections import defaultdict
from threading import Lock

iris_num = None
csv_files = None
csv_writers = None
csv_filenames = None
port_locks = defaultdict(Lock)


def signal_handler(sig, frame):
    global csv_files
    print("\n程序即将退出，正在保存 CSV 文件...")
    for csv_file in csv_files:
        if csv_file:
            csv_file.close()
    print("CSV 文件已保存，程序退出")
    sys.exit(0)

def init_csv():
    global csv_files, csv_writers, csv_filenames, iris_num
    csv_files = [open(csv_filenames[i], "a+", newline="", encoding="utf-8") for i in range(iris_num)]
    csv_writers = [csv.writer(csv_files[i]) for i in range(iris_num)]
    for i in range(iris_num):
        csv_files[i].seek(0)
        if csv_files[i].readline() == "":
            csv_writers[i].writerow(["X", "Y", "Z"])

def main(ports):
    signal.signal(signal.SIGINT, signal_handler)
    init_csv()

    sockets = []
    for port in ports:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8192)
        sock.setblocking(False)
        sock.bind(("0.0.0.0", port))
        sockets.append(sock)

    try:
        while True:
            try:
                global csv_files, csv_writers
                readable, _, _ = select.select(sockets, [], [], 0.01)
                for sock in readable:
                    data, _ = sock.recvfrom(24)
                    port = sock.getsockname()[1]
                    idx = ports.index(port)
                    with port_locks[port]:
                        row = list(struct.unpack('3d', data))
                        csv_writers[idx].writerow(row)
                        csv_files[idx].flush()
            except Exception as e:
                time.sleep(0.001)
    except Exception as e:
        print(f"UDP 服务异常: {e}")
    finally:
        for csv_file in csv_files:
            if csv_file:
                csv_file.close()


if __name__ == "__main__":
    iris_num = int(sys.argv[1])
    csv_filenames = [f"iris_{i}_data.csv" for i in range(iris_num)]
    ports = [9200 + i for i in range(iris_num)]
    main(ports)
