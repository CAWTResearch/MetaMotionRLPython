import pandas as pd

# Cargar datos de sensores (cambiar el nombre del archivo segun sea el caso)
df_sensor_1 = pd.read_csv("sensor_data_CE:94:48:FE:5D:C5.csv")  # Datos del sensor 1
df_sensor_2 = pd.read_csv("sensor_data_EE:1B:72:FA:BF:E8.csv")  # Datos del sensor 2
df_sensor_3 = pd.read_csv("sensor_data_F1:1E:E2:6F:1D:E1.csv")  # Datos del sensor 3
df_sensor_4 = pd.read_csv("sensor_data_F9:8C:1E:4A:F5:D0.csv")  # Datos del sensor 4
df_sensor_5 = pd.read_csv("sensor_data_FA:F1:20:99:CB:B4.csv")  # Datos del sensor 5

# Ordenar los datos por timestamp para asegurar la correcta sincronización
df_sensor_1 = df_sensor_1.sort_values(by='timestamp')
df_sensor_2 = df_sensor_2.sort_values(by='timestamp')
df_sensor_3 = df_sensor_3.sort_values(by='timestamp')
df_sensor_4 = df_sensor_4.sort_values(by='timestamp')
df_sensor_5 = df_sensor_5.sort_values(by='timestamp')

# Sincronizar los datos usando merge_asof()
df_sync = pd.merge_asof(df_sensor_1, df_sensor_2, on='timestamp', suffixes=('_sensor_1', '_sensor_2'))
df_sync = pd.merge_asof(df_sync, df_sensor_3, on='timestamp', suffixes=('', '_sensor_3'))
df_sync = pd.merge_asof(df_sync, df_sensor_4, on='timestamp', suffixes=('', '_sensor_4'))
df_sync = pd.merge_asof(df_sync, df_sensor_5, on='timestamp', suffixes=('', '_sensor_5'))

# Guardar el resultado en un nuevo archivo CSV
df_sync.to_csv("Synchronized_Sensor_Data.csv", index=False)

# Mostrar las primeras filas del dataset sincronizado
print(df_sync.head())
