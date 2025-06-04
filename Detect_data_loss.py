import pandas as pd
import matplotlib.pyplot as plt
import os

def detect_data_loss(csv_file_path, expected_rate):
    df = pd.read_csv(csv_file_path)
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')
    df['second_interval'] = df['sensor_time'].dt.floor('S')
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

    total_intervals_considered = len(counts_filtered)
    expected_total_samples = total_intervals_considered * expected_rate
    actual_total_samples = counts_filtered['sample_count'].sum()
    total_samples_lost = expected_total_samples - actual_total_samples

    return os.path.basename(csv_file_path), total_samples_lost

def analyze_folder(folder_path, expected_rate=50):
    data_loss_results = []

    for filename in os.listdir(folder_path):
        if filename.endswith(".csv"):
            filepath = os.path.join(folder_path, filename)
            file_label, lost_samples = detect_data_loss(filepath, expected_rate)
            data_loss_results.append((file_label, lost_samples))

    return data_loss_results

def plot_data_loss_bar_chart(data_loss_results):
    files = [item[0] for item in data_loss_results]
    losses = [item[1] for item in data_loss_results]

    Average_total_loss = sum(losses)/len(losses) if losses else 0

    plt.figure(figsize=(12, 6))
    plt.bar(files, losses, color='skyblue')
    plt.xticks(rotation=45, ha='right')
    plt.ylabel('Muestras perdidas')
    plt.title('Pérdida de datos por archivo CSV')
    plt.tight_layout()
    plt.grid(axis='y', linestyle='--', alpha=0.7)
    plt.text(0.5, 6, Average_total_loss, f'Pérdida promedio: {Average_total_loss:.2f}')
    plt.show()

if __name__ == "__main__":
    folder = "./your_csv_folder"  # Reemplaza con la ruta de tu carpeta
    results = analyze_folder(folder, expected_rate=50)
    plot_data_loss_bar_chart(results)
