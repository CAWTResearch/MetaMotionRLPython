# plot_data.py
import pandas as pd
import matplotlib.pyplot as plt

cols = [
    'time','timestamp','acc_x','acc_y','acc_z'
]

# 1) Read with the first row as header
df = pd.read_csv(
    'acc_gyro_D5:42:DD:AC:BE:E1.csv', 
    header=0,          
)

df['timestamp_dt'] = pd.to_datetime(df['timestamp'], unit='s')

df['time_of_day'] = pd.to_datetime(df['time'], format='%H:%M:%S.%f')

# make that your index
df.set_index('time_of_day', inplace=True)

print(df.columns.tolist())
print(df.head())

fig, ax = plt.subplots(figsize=(12, 5))
df[['acc_x','acc_y','acc_z']].plot(ax=ax)
ax.set_title("acc vs. Time")
ax.set_xlabel("Time")
ax.set_ylabel("Acc")
plt.tight_layout()
plt.show()
