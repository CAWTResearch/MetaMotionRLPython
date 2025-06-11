import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import timedelta
import os
import glob

def plot_streaming_with_loss(
    csv_file_path: str,
    expected_rate: int = 50,
    sensor_prefix: str = "acc"
):
    # 1) Cargar CSV
    df = pd.read_csv(csv_file_path)

    # 2) Parsear sensor_time a datetime
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')

    # 3) Floor al segundo para agrupar
    df['second_interval'] = df['sensor_time'].dt.floor('s')

    # 4) Determinar primer y último segundo completo
    first_second = df['second_interval'].min()
    last_second  = df['second_interval'].max()

    # 5) Contar cuántas muestras hay en cada segundo
    counts = (
        df.groupby('second_interval')
          .size()
          .reset_index(name='sample_count')
    )

    # 6) Excluir primer y último segundo (posiblemente incompletos)
    counts_filtered = counts[
        (counts['second_interval'] != first_second) &
        (counts['second_interval'] != last_second)
    ].copy()

    # 7) Definir una columna 'color' según la pérdida de datos
    def color_for_count(n):
        if n >= expected_rate:
            return None           # No sombreado si cumple o excede el rate
        elif n >= expected_rate - 5:
            return 'yellow'       # Pérdida ≤ 5 muestras
        elif n >= expected_rate - 10:
            return 'orange'       # Pérdida 6–10 muestras
        else:
            return 'red'          # Pérdida > 10 muestras

    counts_filtered['color'] = counts_filtered['sample_count'].apply(color_for_count)

    # 8) Preparar el plot
    fig, ax = plt.subplots(figsize=(14, 5))

    # 9) Graficar las tres componentes (x, y, z) vs sensor_time
    ax.plot(df['sensor_time'], df[f'{sensor_prefix}_x'], label=f'{sensor_prefix}_x', color='C0', linewidth=0.8)
    ax.plot(df['sensor_time'], df[f'{sensor_prefix}_y'], label=f'{sensor_prefix}_y', color='C1', linewidth=0.8)
    ax.plot(df['sensor_time'], df[f'{sensor_prefix}_z'], label=f'{sensor_prefix}_z', color='C2', linewidth=0.8)

    # 10) Sombrear cada segundo con pérdida según su color
    for _, row in counts_filtered.iterrows():
        sec_start = row['second_interval']
        sec_end   = sec_start + timedelta(seconds=1)
        c = row['color']
        if c is not None:
            ax.axvspan(sec_start, sec_end, color=c, alpha=0.3)

    # 12) Formato del eje X: mostrar hora:minuto:segundo
    locator = mdates.AutoDateLocator(minticks=6, maxticks=12)
    formatter = mdates.DateFormatter('%H:%M:%S')
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(formatter)

    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # 13) Etiquetas y leyenda
    ax.set_title(f"{sensor_prefix.upper()}{csv_file_path} Data (Streaming) - Sombras indican pérdida de datos")
    ax.set_xlabel("Tiempo (sensor_time)")
    ax.set_ylabel("Valor")
    ax.legend(loc='upper right', fontsize='small')

    # 14) Ajustar márgenes y mostrar
    plt.tight_layout()
    plt.show()


def batch_plot(folder: str, sensor_prefix: str, expected_rate: int = 50):
    """
    Finds every CSV in `folder` named sensor_prefix_*.csv
    and calls plot_streaming_with_loss on each.
    """
    pattern = os.path.join(folder, f"{sensor_prefix}_*.csv")
    for path in sorted(glob.glob(pattern)):
        print(f"\n=== Plotting {sensor_prefix.upper()} data: {os.path.basename(path)} ===")
        plot_streaming_with_loss(
            csv_file_path=path,
            expected_rate=expected_rate,
            sensor_prefix=sensor_prefix
        )

if __name__ == "__main__":
    data_folder   = "DriveUpload"  # folder where your acc_*.csv and gyro_*.csv live
    expected_rate = 49             # or whichever rate you need

    # Plot all ACC files, then all GYRO files
    batch_plot(data_folder, "acc", expected_rate)
    batch_plot(data_folder, "gyro", expected_rate)