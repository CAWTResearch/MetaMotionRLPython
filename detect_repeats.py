import pandas as pd
import os

# Load the CSV into a DataFrame
df = pd.read_csv('DriveUpload/combined_data.csv')

# Determine if each row is identical to the previous row
df['is_repeat'] = df.eq(df.shift()).all(axis=1)
repeat_count = int(df['is_repeat'].sum())

# For convenience, fill the first row's repeat flag as False
df.loc[0, 'is_repeat'] = False

# Save the result to a new CSV
output_path = 'DriveUpload/repeat_detection.csv'
df.to_csv(output_path, index=False)

print("Total repeat rows:", repeat_count)
print(f"Processed {len(df)} rows. Results saved to '{output_path}'.")
print(f"Percent of repeats: {repeat_count/len(df)}")

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

    # Output results
    print(f"Average interval: {avg_interval:.6f} seconds")
    print(f"Minimum interval: {min_interval:.6f} seconds")
    print(f"Maximum interval: {max_interval:.6f} seconds")

if __name__ == '__main__':
    main()
