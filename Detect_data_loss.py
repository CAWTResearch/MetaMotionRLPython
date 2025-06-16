import pandas as pd
import matplotlib.pyplot as plt
import os

def detect_data_loss(csv_file_path, expected_rate):
    df = pd.read_csv(csv_file_path)
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')
    df['second_interval'] = df['sensor_time'].dt.floor('s')
    first_second = df['second_interval'].min()
    last_second = df['second_interval'].max()

    counts = (
        df.groupby('second_interval')
          .size()
          .reset_index(name='sample_count')
    )

    counts_filtered = counts[
        (counts['second_interval'] != first_second) &
        (counts['second_interval'] != last_second)
    ]

    loss_intervals = counts_filtered[counts_filtered['sample_count'] < expected_rate]

    # 2) Seconds entirely missing from the data
    full_seconds = pd.date_range(first_second, last_second, freq='s').floor('s')

    # Only drop the *actual* seconds that exist in first/last rows
    actual_first = df['second_interval'].iloc[0]
    actual_last  = df['second_interval'].iloc[-1]
    full_seconds = [ts for ts in full_seconds if ts != actual_first and ts != actual_last]

    present = set(counts['second_interval'])
    missing = [ts for ts in full_seconds if ts not in present]
    if missing:
        print(f"\nAlso, these seconds were completely missing from {os.path.basename(csv_file_path)}:")
        for ts in missing:
            print(f"  • {ts.time()}")
    else:
        print(f"\nNo fully-missing seconds in {os.path.basename(csv_file_path)}.")

    # # → PRINT intervals with fewer-than-expected samples
    # low = counts_filtered[counts_filtered['sample_count'] < expected_rate]
    # if not low.empty:
    #     print(f"\n[{os.path.basename(csv_file_path)}] Segundos con < {expected_rate} muestras:")
    #     for _, r in low.iterrows():
    #         print(f"  • {r['second_interval'].time()} → {r['sample_count']} muestras")
    # else:
    #     print(f"\n[{os.path.basename(csv_file_path)}] No segundos con < {expected_rate} muestras.")



    total_intervals_considered = len(counts_filtered)
    expected_total_samples = total_intervals_considered * expected_rate
    actual_total_samples = counts_filtered['sample_count'].sum()
    total_samples_lost = expected_total_samples - actual_total_samples


    return os.path.basename(csv_file_path), total_samples_lost, expected_total_samples

def analyze_folder(folder_path, expected_rate=50):
    data_loss_results = []

    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            filepath = os.path.join(folder_path, filename)
            file_label, lost_samples, expected_samples = detect_data_loss(filepath, expected_rate)
            data_loss_results.append((file_label, lost_samples, expected_samples))

    return data_loss_results

def plot_data_loss_bar_chart(data_loss_results):
    files = [item[0] for item in data_loss_results]
    losses = [item[1] for item in data_loss_results]
    expected_samples = [item[2] for item in data_loss_results]

    total_loss = sum(losses)
    total_expected_samples = sum(expected_samples)

    Porcentaje_Data = (total_expected_samples - total_loss)/ total_expected_samples

    plt.figure(figsize=(12, 6))
    plt.bar(files, losses, color='skyblue')
    plt.xticks(rotation=45, ha='right')
    plt.ylabel('Muestras perdidas')
    plt.suptitle('Pérdida de datos por archivo CSV')
    plt.title('Porcentaje de Data: ' + str(Porcentaje_Data))
    plt.tight_layout()
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.show()

if __name__ == "__main__":
    folder = "./1.5meterblanky30minutetest3"  # Reemplaza con la ruta de tu carpeta
    results = analyze_folder(folder, expected_rate=50)
    plot_data_loss_bar_chart(results)
