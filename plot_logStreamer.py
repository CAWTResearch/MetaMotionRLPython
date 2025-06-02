#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import sys


def main(csv_path):
    # Load the CSV, parse the first column as datetime and set it as index
    df = pd.read_csv(
        csv_path,
        parse_dates=[0],       # parse first column (timestamp) as datetime
        index_col=0            # set first column as index
    )
    df.index.name = 'time'    # rename index for clarity

    # Inspect
    print("Index set to:", df.index.name)
    print("Data columns:", df.columns.tolist())
    print(df.head())

    # Plot all data columns vs time
    fig, ax = plt.subplots(figsize=(12, 5))
    df.plot(ax=ax)
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

