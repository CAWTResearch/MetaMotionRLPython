#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys

def main(csv_path):
    # 1) Load the CSV, let pandas infer the columns
    df = pd.read_csv(csv_path, header=0)
    
    # 2) Identify the time‐column (first one)
    time_col = df.columns[0]

    # 3) Parse that column as datetime
    df[time_col] = pd.to_datetime(
        df[time_col],
        format='%Y-%m-%d %H:%M:%S.%f',
        errors='raise'
    )
    
    # 4) Rename it to a uniform name and set as index
    df.rename(columns={time_col: 'time'}, inplace=True)
    df.set_index('time', inplace=True)

    # 5) Inspect
    print("Index set to:", df.index.name)
    print("Data columns:", df.columns.tolist())
    print(df.head())

    # 6) Plot all remaining columns
    fig, ax = plt.subplots(figsize=(12, 5))
    df.plot(y=df.columns.tolist(), ax=ax)
    ax.set_title(f"{', '.join(df.columns)} vs. Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Value")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plotlog.py <path_to_csv>")
        sys.exit(1)
    main(sys.argv[1])


# Run: python3 plotlog.py acc_D542DDACBEE1.csv 
# Run: python3 plotlog.py gyro_D542DDACBEE1.csv 

