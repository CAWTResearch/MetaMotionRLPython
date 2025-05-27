import pandas as pd
import matplotlib.pyplot as plt

# 1) Read CSV (header row contains the column names)
df = pd.read_csv(
    'acc_gyro_F768558D840E.csv', 
    header=0,          
)

# 2) Convert epoch_ms (milliseconds since Unix epoch) to datetime
df['time'] = pd.to_datetime(df['epoch_ms'], unit='ms')

# 3) Set the new datetime column as the DataFrame index
df.set_index('time', inplace=True)

# 4) (Optional) Inspect the DataFrame
print(df.columns.tolist())
print(df.head())

# 5) Plot accelerometer channels vs. time
fig, ax = plt.subplots(figsize=(12, 5))
df[['acc_x', 'acc_y', 'acc_z']].plot(ax=ax)
ax.set_title("Acceleration vs. Time")
ax.set_xlabel("Time")
ax.set_ylabel("Acceleration (g)")
plt.tight_layout()
plt.show()
