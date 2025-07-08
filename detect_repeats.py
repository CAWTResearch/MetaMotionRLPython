import pandas as pd
import os

# Load the CSV into a DataFrame
df = pd.read_csv('DriveUpload/Synchronized_Sensor_Data.csv')
# df = pd.read_csv('DriveUpload/combined_data.csv')
print(f"Total rows in combined_data.csv: {len(df)}")

# 1) Identify only the sensor columns that start with 'n'
sensor_cols = [c for c in df.columns if c.startswith('n')]

# 2) For each of those sensors, compare to the value in the previous row
for sensor in sensor_cols:
    # Boolean column: True if this row's sample == previous row's sample
    df[f'{sensor}_is_repeat'] = df[sensor].eq(df[sensor].shift())
    # df[f'{sensor}_is_repeat'] = df[sensor].eq(400)
    # First row has no “previous”—mark it False
    df.loc[0, f'{sensor}_is_repeat'] = False

# 3) (Optional) Count repeats per sensor
repeat_counts = {
    sensor: int(df[f'{sensor}_is_repeat'].sum())
    for sensor in sensor_cols
}

# 4) (Optional) Percent of repeats per sensor
repeat_percents = {
    sensor: repeat_counts[sensor] / (len(df) - 1)  # exclude first row
    for sensor in sensor_cols
}

# Print summary
for sensor in sensor_cols:
    print(f"{sensor}: {repeat_counts[sensor]} repeats, "
          f"{repeat_percents[sensor]:.2%} of intervals")

# 5) (Optional) If you still want a single “any-n-repeat” column:
df['any_n_repeat'] = df[[f'{sensor}_is_repeat' for sensor in sensor_cols]].any(axis=1)

print("Total rows where any 'n' sensor repeated:", int(df['any_n_repeat'].sum()))
print("Percent of rows with any 'n' repeat:", df['any_n_repeat'].mean())

def main():
    # Path to your predictions CSV
    csv_path = os.path.join('DriveUpload', 'predictions.csv')
    if not os.path.exists(csv_path):
        print(f"File not found: {csv_path}")
        return

    # Read CSV and parse timestamps
    df = pd.read_csv(csv_path, parse_dates=['timestamp'])

    # Compute time differences (in seconds) between consecutive predictions
    df['delta'] = df['timestamp'].diff().dt.total_seconds()

    # Drop the first NaN delta
    deltas = df['delta'].dropna()

    # Calculate statistics
    avg_interval = deltas.mean()
    min_interval = deltas.min()
    max_interval = deltas.max()

    total_duration = (df['timestamp'].iloc[-1] - df['timestamp'].iloc[0]).total_seconds() + 1

    # Output results
    print(f"Average interval: {avg_interval:.6f} seconds")
    print(f"Minimum interval: {min_interval:.6f} seconds")
    print(f"Maximum interval: {max_interval:.6f} seconds")
    print(f"Total test duration: {total_duration:.6f} seconds")

if __name__ == '__main__':
    main()
