import pandas as pd

def detect_data_loss(csv_file_path, expected_rate):
    """
    Lee un CSV con una columna “sensor_time” a 50 Hz (por defecto),
    detecta los intervalos de un segundo con menos muestras de las esperadas,
    ignora el primer y el último segundo (porque suelen estar incompletos),
    suma cuántas muestras faltan en total y lo imprime,
    además de indicar cuántas muestras se esperaban en esos intervalos.
    """
    # 1. Cargar el CSV
    df = pd.read_csv(csv_file_path)

    # 2. Parsear sensor_time a datetime (solo hora)
    df['sensor_time'] = pd.to_datetime(df['sensor_time'], format='%H:%M:%S.%f')

    # 3. Redondear hacia abajo cada timestamp al segundo más próximo
    df['second_interval'] = df['sensor_time'].dt.floor('S')

    # 4. Obtener el primer y el último segundo en todo el registro
    first_second = df['second_interval'].min()
    last_second  = df['second_interval'].max()

    # 5. Contar cuántas muestras hay en cada intervalo de un segundo
    counts = (
        df.groupby('second_interval')
          .size()
          .reset_index(name='sample_count')
    )

    # 6. Filtrar para ignorar el primer y el último segundo
    counts_filtered = counts[
        (counts['second_interval'] != first_second) &
        (counts['second_interval'] != last_second)
    ]

    # 7. Identificar intervalos donde sample_count < expected_rate
    loss_intervals = counts_filtered[counts_filtered['sample_count'] < expected_rate]

    # 8. Variables de conteo
    num_intervals_with_loss = 0
    total_samples_lost = 0

    # 9. Cálculos adicionales: totales esperados y totales reales
    total_intervals_considered = len(counts_filtered)
    expected_total_samples = total_intervals_considered * expected_rate
    actual_total_samples = counts_filtered['sample_count'].sum()

    # 10. Impresión de resultados detallados
    print(
        "Intervalos de segundo con pérdida de datos "
        f"(menos de {expected_rate} muestras),\n"
        f"ignorado el primer segundo ({first_second.time().strftime('%H:%M:%S')}) "
        f"y el último segundo ({last_second.time().strftime('%H:%M:%S')}):"
    )

    if loss_intervals.empty:
        print(f"  Ninguno. Todos los intervalos considerados tienen al menos {expected_rate} muestras.")
    else:
        for _, row in loss_intervals.iterrows():
            t_str = row['second_interval'].time().strftime("%H:%M:%S")
            missing = expected_rate - row['sample_count']
            num_intervals_with_loss += 1
            total_samples_lost += missing
            # print(f"  {t_str} – Muestras: {row['sample_count']}  → Faltan: {missing}")

    # 11. Imprimir totales
    print()
    print(f"Total de intervalos considerados (excluyendo primero y último): {total_intervals_considered}")
    print(f"Total de muestras esperadas en esos intervalos: {expected_total_samples}")
    print(f"Total de muestras reales observadas en esos intervalos: {actual_total_samples}")
    print(f"Total de intervalos con pérdida: {num_intervals_with_loss}")
    print(f"Porcentaje de data actual: {actual_total_samples/expected_total_samples}")
    print(f"Total de muestras faltantes en todo el registro (excluyendo primero/último): {total_samples_lost}")


if __name__ == "__main__":
    # Reemplaza 'TrueTestStreamAcc1.csv' con la ruta a tu archivo CSV real
    detect_data_loss('gyro_C4_65_87_1A_13_0B.csv', expected_rate=50)
    detect_data_loss('gyro_CE_5A_39_E6_8F_B3.csv', expected_rate=50)
    detect_data_loss('gyro_D5_42_DD_AC_BE_E1.csv', expected_rate=50)
    detect_data_loss('gyro_E6_4F_B9_D7_18_7C.csv', expected_rate=50)
    detect_data_loss('gyro_E6_AC_5E_B8_4C_D9.csv', expected_rate=50)
    detect_data_loss('gyro_F0_3D_E7_ED_F6_F7.csv', expected_rate=50)
    
    
