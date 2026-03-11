import sys
import copy
import numpy as np
from datetime import datetime
import pandas as pd
import matplotlib.pyplot as plt
import os
import re


def find_max_num_csv_list(folder_path, recursive=False):
    if not os.path.isdir(folder_path):
        print(f"错误：文件夹路径 {folder_path} 不存在！")
        return []
    
    pattern = re.compile(r'^.*_(\d+)\.csv$')
    num_file_list = []
    
    if recursive:
        for root, dirs, files in os.walk(folder_path):
            for filename in files:
                match = pattern.match(filename)
                if match:
                    num = int(match.group(1))
                    full_path = os.path.join(root, filename)
                    num_file_list.append((num, full_path))
    else:
        for filename in os.listdir(folder_path):
            match = pattern.match(filename)
            if match:
                num = int(match.group(1))
                full_path = os.path.join(folder_path, filename)
                num_file_list.append((num, full_path))
    
    if not num_file_list:
        print("提示：未找到形如**_数字.csv的文件！")
        return []
    
    all_nums = [item[0] for item in num_file_list]
    max_num = max(all_nums)
    max_file_list = [item[1] for item in num_file_list if item[0] == max_num]
    return max_file_list


def get_same_end_char_intervals(str_array):
    end_chars = []
    for s in str_array:
        end_char = s[-1]
        if end_char not in {'0', '1', '2', '3'}:
            return None
        end_chars.append(end_char)
    intervals = []
    current_char = end_chars[0]
    start_idx = 0
    for idx in range(1, len(end_chars)):
        if end_chars[idx] != current_char:
            intervals.append({
                'end_char': current_char,
                'start_idx': start_idx,
                'end_idx': idx - 1
            })
            current_char = end_chars[idx]
            start_idx = idx
    intervals.append({
        'end_char': current_char,
        'start_idx': start_idx,
        'end_idx': len(end_chars) - 1
    })
    return intervals


def read_csv(csv_path):
    df = pd.read_csv(csv_path)
    column_count = len(df.columns)
    if column_count == 15:
        col1 = df.iloc[:, 1].to_numpy()
        col2 = df.iloc[:, 2].to_numpy()
        col3 = df.iloc[:, 3].to_numpy()
        col4 = df.iloc[:, 4].to_numpy()
        col5 = df.iloc[:, 0].to_numpy()
        return copy.deepcopy([col1, col2, col3, col4, col5])
    else:
        col1 = df.iloc[:, 0].to_numpy()
        col2 = df.iloc[:, 1].to_numpy()
        col3 = df.iloc[:, 2].to_numpy()
        col4 = df.iloc[:, 3].to_numpy()
        return copy.deepcopy([col1, col2, col3, col4])


def plot_single(time_list, data_list, xyz, highlight_intervals, labels, paths):
    start = [p.rfind('/') for p in paths]
    mid = [p.rfind('_') for p in paths]
    end = [p.rfind('.') for p in paths]
    plt.figure(figsize=(12, 6))
    ax = plt.gca()
    for i in range(len(time_list)):
        if len(time_list[i]) != 0:
            x_min, x_max, y_min, y_max = time_list[i].min(), time_list[i].max(), data_list[i].min(), data_list[i].max()
            break
    for t, x, p, s, m in zip(time_list, data_list, paths, start, mid):
        if len(t) == 0:
            continue
        ax.plot(t, x, label=p[s + 1: m])
        x_min = min(x_min, t.min())
        x_max = max(x_max, t.max())
        y_min = min(y_min, x.min())
        y_max = max(y_max, x.max())
    ax.set_xlim(x_min - x_min - 0.5, x_max - x_min + 0.5)
    ax.set_ylim(y_min - 0.2, y_max + 0.4)
    if len(highlight_intervals) > 0:
        for i, interval in enumerate(highlight_intervals):
            ax.axvspan(interval[0] - x_min, interval[1] - x_min, alpha=0.2, color='red')
            ax.text(
                (interval[0] - x_min + interval[1] - x_min) / 2,
                y_max + 0.2,
                labels[i],
                ha='center',
                va='center',
                color='darkred',
                fontsize=12,
                fontweight='bold'
            )
    ax.set_title(f'{xyz} direction figure')
    ax.set_xlabel('T')
    ax.set_ylabel(xyz)
    ax.legend(loc='lower left')
    plt.tight_layout()
    plt.savefig(f'{paths[0][:start[0]]}/{xyz}_{paths[0][mid[0] + 1: end[0]]}.png', dpi=300, bbox_inches='tight')
    plt.show()


def plot_all(recursive=False):
    csv_path_list = find_max_num_csv_list(f"{os.environ.get('HOME')}/DataRecord/{datetime.now().strftime('%Y%m%d')}/PositionData", recursive)
    time_list = []
    x_list = []
    y_list = []
    z_list = []
    attack_dict = {
        '1': 'DoS attack',
        '2': 'FDI attack',
        '3': 'replay attack'
    }
    highlight_intervals = []
    labels = []
    for csv_path in csv_path_list:
        data = copy.deepcopy(read_csv(csv_path))
        time_list.append(copy.deepcopy(data[0]))
        x_list.append(copy.deepcopy(data[1]))
        y_list.append(copy.deepcopy(data[2]))
        z_list.append(copy.deepcopy(data[3]))
        if len(data) == 5 and len(highlight_intervals) == 0:
            intervals = get_same_end_char_intervals(copy.deepcopy(data[4]))
            for interval in intervals:
                if interval['end_char'] != '0':
                    highlight_intervals.append([data[0][interval['start_idx']], data[0][interval['end_idx']]])
                    labels.append(attack_dict[interval['end_char']])
    plot_single(time_list, x_list, 'X', highlight_intervals, labels, csv_path_list)
    plot_single(time_list, y_list, 'Y', highlight_intervals, labels, csv_path_list)
    plot_single(time_list, z_list, 'Z', highlight_intervals, labels, csv_path_list)


if __name__ == "__main__":
    plot_all()
