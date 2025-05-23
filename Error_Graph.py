import pandas as pd
import matplotlib.pyplot as plt

# 
df = pd.read_csv("tracking_error_log.csv")

# 
plt.figure(figsize=(12, 6))

# Error Plot
plt.subplot(2, 1, 1)
plt.plot(df["Time(s)"], df["Error"], label="Error", color='blue')
plt.axhline(0, color='gray', linestyle='--')
plt.title("Tracking Error over Time")
plt.xlabel("Time (s)")
plt.ylabel("Error (pixels)")
plt.grid(True)

# PID Output Plot
plt.subplot(2, 1, 2)
plt.plot(df["Time(s)"], df["PID_Output"], label="PID Output", color='orange')
plt.axhline(0, color='gray', linestyle='--')
plt.title("PID Output over Time")
plt.xlabel("Time (s)")
plt.ylabel("PID Output")
plt.grid(True)

plt.tight_layout()
plt.show()
