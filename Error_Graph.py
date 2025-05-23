# -*- coding: utf-8 -*-

import matplotlib
matplotlib.use('Agg')  # GUI 없이 파일로 저장

import matplotlib.pyplot as plt
import pandas as pd
import codecs

# CSV 파일을 UTF-8로 안전하게 읽기 (Python 2 대응)
with codecs.open("tracking_error_log.csv", 'r', 'utf-8') as f:
    df = pd.read_csv(f)

# 그래프 출력 준비
plt.figure(figsize=(12, 6))

# 1. Error Plot
plt.subplot(2, 1, 1)
plt.plot(df["Time(s)"], df["Error"], label="Error", color='blue')
plt.axhline(0, color='gray', linestyle='--')
plt.title("Tracking Error over Time")
plt.xlabel("Time (s)")
plt.ylabel("Error (pixels)")
plt.grid(True)

# 2. PID Output Plot
plt.subplot(2, 1, 2)
plt.plot(df["Time(s)"], df["PID_Output"], label="PID Output", color='orange')
plt.axhline(0, color='gray', linestyle='--')
plt.title("PID Output over Time")
plt.xlabel("Time (s)")
plt.ylabel("PID Output")
plt.grid(True)

# 레이아웃 정리 및 파일로 저장
plt.tight_layout()
plt.savefig("tracking_result_plot.png")
