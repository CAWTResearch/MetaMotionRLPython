# plot_data.py
import pandas as pd
import matplotlib.pyplot as plt

cols = [
    'time','timestamp','quat_w','quat_x','quat_y','quat_z',
]

# 1) Read with the first row as header
df = pd.read_csv(
    'sensor_data_F7:68:55:8D:84:0E.csv', 
    header=0,          
)

df['timestamp_dt'] = pd.to_datetime(df['timestamp'], unit='s')

df['time_of_day'] = pd.to_datetime(df['time'], format='%H:%M:%S.%f')

# make that your index
df.set_index('time_of_day', inplace=True)

print(df.columns.tolist())
print(df.head())

fig, ax = plt.subplots(figsize=(12, 5))
df[['quat_w','quat_x','quat_y','quat_z']].plot(ax=ax)
ax.set_title("QUAT vs. Time")
ax.set_xlabel("Time")
ax.set_ylabel("QUAT")
plt.tight_layout()
plt.show()
