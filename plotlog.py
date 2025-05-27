#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys

def main(csv_path):
    # 1) Load the CSV, parsing the first column as dates
    df = pd.read_csv(
        csv_path,
        header=0,
        parse_dates=['epoch_ms'],       # tell pandas this column is dates
        date_parser=lambda s: pd.to_datetime(s, format='%Y-%m-%d %H:%M:%S.%f')
    )

    # 2) Rename for clarity
    df.rename(columns={'epoch_ms':'time'}, inplace=True)

    # 3) Make it the index
    df.set_index('time', inplace=True)

    # 4) Inspect
    print("Columns:", df.columns.tolist())
    print(df.head())

    # 5) Plot
    fig, ax = plt.subplots(figsize=(12, 5))
    df[['acc_x', 'acc_y', 'acc_z']].plot(ax=ax)
    ax.set_title("Acceleration vs. Time")
    ax.set_xlabel("Time")
    ax.set_ylabel("Acceleration (g)")
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python plotlog.py <acc_csv_file>")
        sys.exit(1)
    main(sys.argv[1])

# Run: python3 plotlog.py acc_D542DDACBEE1.csv 
# Run: python3 plotlog.py acc_D542DDACBEE1.csv 

