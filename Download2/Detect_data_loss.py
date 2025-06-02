import pandas as pd

def detect_data_loss(csv_file_path, expected_rate, count, total_loss):
    """
    Reads a CSV with a “sensor_time” column at 50 Hz, 
    then prints which one-second intervals have fewer than `expected_rate` samples.
    """
    # 1. Load the CSV file
    df = pd.read_csv(csv_file_path)

    # 2. Parse sensor_time into a datetime (time-of-day)
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')

    # 3. Floor each timestamp to the nearest whole second
    df['second_interval'] = df['sensor_time'].dt.floor('S')

    # 4. Count how many samples fall into each one-second interval
    counts = df.groupby('second_interval').size().reset_index(name='sample_count')

    # 5. Identify intervals where sample_count < expected_rate (50 by default)
    loss = counts[counts['sample_count'] < expected_rate]

    # 6. Print results
    print(f"Second intervals with data loss (fewer than {expected_rate} samples):")
    if loss.empty:
        print(f"  None. Every one-second interval has at least {expected_rate} samples.")
    else:
        for _, row in loss.iterrows():
            t = row['second_interval'].time().strftime("%H:%M:%S")
            count+=1
            # total_loss+=expected_rate-counts+1
            print(f"  {t} - Samples: {row['sample_count']} Count: "+str(count))

if __name__ == "__main__":
    # Replace 'your_data.csv' with the path to your CSV file
    detect_data_loss('acc_CE_5A_39_E6_8F_B3.csv', expected_rate=49, count=0, total_loss=0)
