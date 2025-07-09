#!/usr/bin/env python
import pandas as pd
import numpy as np
import sys
import glob
from datetime import datetime

def load_csv(csv_path):
    # Try reading with header inference
    df = pd.read_csv(csv_path)
    # If the first column name is literally "timestamp", assume there was a header
    if df.columns[0].lower() == 'timestamp':
        # Ensure we have exactly the columns we expect
        expected_cols = ['timestamp', 'true_label', 'p0','p1','p2','p3','p4','p5']
        if list(df.columns[:8]) != expected_cols:
            # Rename the first 8 columns in case of extra columns
            df = df.rename(columns=dict(zip(df.columns[:8], expected_cols)))
        return df[expected_cols]
    else:
        # No header row: read again, assigning our own names
        return pd.read_csv(
            csv_path,
            header=None,
            names=['timestamp', 'true_label', 'p0','p1','p2','p3','p4','p5'],
            usecols=range(8)
        )

def detect_accuracy(csv_path):
    # Load data, handling header or no-header
    df = load_csv(csv_path)

    # Parse timestamps
    try:
        df['timestamp'] = pd.to_datetime(df['timestamp'])
    except Exception as e:
        print(f"Error parsing timestamps: {e}")
        sys.exit(1)

    # Compute elapsed seconds from the first sample
    start_time = df['timestamp'].iloc[0]
    df['elapsed'] = (df['timestamp'] - start_time).dt.total_seconds()

    # Define our six 60-second intervals (with 15-second gaps)
    intervals = [
        (0,   60),
        (75, 135),
        (150,210),
        (225,285),
        (300,360),
        (375,435),
    ]

    # Map each elapsed timestamp to its expected label
    def get_expected(elapsed):
        for label, (start, end) in enumerate(intervals):
            if start <= elapsed <= end:
                return label
        return None

    df['expected'] = df['elapsed'].apply(get_expected)
    df = df.dropna(subset=['expected']).copy()
    df['expected'] = df['expected'].astype(int)

    # Determine predicted label via argmax over p0…p5
    prob_cols = ['p0','p1','p2','p3','p4','p5']
    df['predicted'] = np.argmax(df[prob_cols].values, axis=1)

    # Calculate accuracy per interval
    results = []
    for label, (start, end) in enumerate(intervals):
        mask = df['expected'] == label
        total = mask.sum()
        if total:
            correct = (df.loc[mask, 'predicted'] == label).sum()
            accuracy = correct / total * 100
            results.append({
                'interval': f'{start}-{end}s',
                'label':     label,
                'accuracy_%': round(accuracy, 3)
            })

    # Print a neat summary
    print(f"\nAccuracy results for '{csv_path}':\n")
    print(f"{'Interval':>10}  {'Label':>5}  {'Accuracy (%)':>12}")
    print("-"*32)
    for row in results:
        print(f"{row['interval']:>10}  {row['label']:>5}  {row['accuracy_%']:>12.2f}")
    print()

def choose_csv():
    csvs = glob.glob("predictions.csv")
    if not csvs:
        print("No CSV files found in current directory.")
        sys.exit(1)
    return csvs[0]

if _name_ == '_main_':
    if len(sys.argv) == 2:
        path = sys.argv[1]
    elif len(sys.argv) == 1:
        path = choose_csv()
        print(f"Auto-detected CSV: {path}")
    else:
        print("Usage: python detect_accuracy.py [path/to/your.csv]")
        sys.exit(1)

    detect_accuracy(path)