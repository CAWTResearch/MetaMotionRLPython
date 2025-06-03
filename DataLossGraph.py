import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.dates as mdates
from datetime import timedelta

def plot_streaming_with_loss(
    csv_file_path: str,
    expected_rate: int = 50,
    sensor_prefix: str = "acc"
):
    """
    Genera un gráfico de líneas para las tres componentes (sensor_prefix_x, sensor_prefix_y, sensor_prefix_z)
    versus sensor_time, y sombrea en distintos colores los segundos donde hubo pérdida de datos (excluyendo
    el primer y el último segundo completo).

    Parámetros:
    - csv_file_path: ruta al CSV de streaming que debe contener al menos las columnas:
        • sensor_time  (formato 'HH:MM:SS.%f')
        • {sensor_prefix}_x, {sensor_prefix}_y, {sensor_prefix}_z
    - expected_rate: muestras esperadas por segundo (por defecto 50)
    - sensor_prefix: prefijo de las columnas de sensor ('acc' o 'gyro', por ejemplo)

    La paleta de colores para cada segundo (basado en cuántas muestras realmente llegaron) es:
      • sample_count >= expected_rate: sin sombreado (ninguna pérdida)
      • sample_count >= (expected_rate - 5) [p. ej. 45–49]: amarillo (pérdida ≤ 5)
      • sample_count >= (expected_rate - 10) [p. ej. 40–44]: naranja (pérdida 6–10)
      • sample_count <  (expected_rate - 10) [menos de 40 muestras]: rojo (pérdida > 10)
    """

    # 1) Cargar CSV
    df = pd.read_csv(csv_file_path)

    # 2) Parsear sensor_time a datetime
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')

    # 3) Floor al segundo para agrupar
    df['second_interval'] = df['sensor_time'].dt.floor('S')

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

    # 11) Línea de referencia en expected_rate (opcional, si se quiere ver el conteo)
    #    Si no interesa, este bloque puede comentarse.
    # total_intervals = len(counts_filtered)
    # expected_line_y = expected_rate  # este valor solo sirve si se graficara el conteo de muestras/segundo
    # ax.axhline(expected_line_y, linestyle='--', color='k', linewidth=0.7, label=f'Esperadas ({expected_rate})')

    # 12) Formato del eje X: mostrar hora:minuto:segundo
    locator = mdates.AutoDateLocator(minticks=6, maxticks=12)
    formatter = mdates.DateFormatter('%H:%M:%S')
    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(formatter)

    plt.setp(ax.xaxis.get_majorticklabels(), rotation=45, ha='right')

    # 13) Etiquetas y leyenda
    ax.set_title(f"{sensor_prefix.upper()} Data (Streaming) - Sombras indican pérdida de datos")
    ax.set_xlabel("Tiempo (sensor_time)")
    ax.set_ylabel("Valor")
    ax.legend(loc='upper right', fontsize='small')

    # 14) Ajustar márgenes y mostrar
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    # Ejemplo de uso para acc:
    # plot_streaming_with_loss(
    #     csv_file_path="TrueTestLogAcc1.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    plot_streaming_with_loss(
        csv_file_path="gyro_C4_65_87_1A_13_0B.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )
    plot_streaming_with_loss(
        csv_file_path="gyro_CE_5A_39_E6_8F_B3.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )
    plot_streaming_with_loss(
        csv_file_path="gyro_D5_42_DD_AC_BE_E1.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )
    plot_streaming_with_loss(
        csv_file_path="gyro_E6_4F_B9_D7_18_7C.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )
    plot_streaming_with_loss(
        csv_file_path="gyro_E6_AC_5E_B8_4C_D9.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )
    plot_streaming_with_loss(
        csv_file_path="gyro_F0_3D_E7_ED_F6_F7.csv",
        expected_rate=49,
        sensor_prefix="gyro"
    )

    # plot_streaming_with_loss(
    #     csv_file_path="acc_C4.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    # plot_streaming_with_loss(
    #     csv_file_path="acc_CE_5A_39_E6_8F_B3.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    # plot_streaming_with_loss(
    #     csv_file_path="acc_D5_42_DD_AC_BE_E1.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    # plot_streaming_with_loss(
    #     csv_file_path="acc_E6_4F_B9_D7_18_7C.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    # plot_streaming_with_loss(
    #     csv_file_path="acc_E6_AC_5E_B8_4C_D9.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )
    # plot_streaming_with_loss(
    #     csv_file_path="acc_F0_3D_E7_ED_F6_F7.csv",
    #     expected_rate=49,
    #     sensor_prefix="acc"
    # )

    # Ejemplo de uso para gyro:
    # plot_streaming_with_loss(
    #     csv_file_path="ruta/a/tu_streaming_gyro.csv",
