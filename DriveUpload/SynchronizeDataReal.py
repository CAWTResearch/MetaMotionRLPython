import pandas as pd

# Cargar datos de sensores (cambiar el nombre del archivo segun sea el caso)

device_macs = ["CE:5A:39:E6:8F:B3", "F7:68:55:8D:84:0E", "F8:DC:C7:F1:48:7A", "E6:4F:B9:D7:18:7C", "F0:3D:E7:ED:F6:F7", "F4:73:A1:AB:BB:64"]



desired_cols = [
    'host_time',
    'q_chest_w', 'q_chest_x', 'q_chest_y', 'q_chest_z',
    'q_left_hand_w', 'q_left_hand_x', 'q_left_hand_y', 'q_left_hand_z',
    'q_right_knee_w', 'q_right_knee_x', 'q_right_knee_y', 'q_right_knee_z',
    'n_chest_acc_x', 'n_chest_acc_y', 'n_chest_acc_z',
    'n_chest_gyro_x', 'n_chest_gyro_y', 'n_chest_gyro_z',
    'n_left_knee_acc_x', 'n_left_knee_acc_y', 'n_left_knee_acc_z',
    'n_left_knee_gyro_x', 'n_left_knee_gyro_y', 'n_left_knee_gyro_z',
    'n_right_hand_acc_x', 'n_right_hand_acc_y', 'n_right_hand_acc_z',
    'n_right_hand_gyro_x', 'n_right_hand_gyro_y', 'n_right_hand_gyro_z'
]




df_n_chest_acc = pd.read_csv("acc_CE:5A:39:E6:8F:B3.csv")
df_n_chest_gyro = pd.read_csv("gyro_CE:5A:39:E6:8F:B3.csv")

df_n_left_knee_acc = pd.read_csv("acc_F4:73:A1:AB:BB:64.csv")
df_n_left_knee_gyro = pd.read_csv("gyro_F4:73:A1:AB:BB:64.csv")

df_n_right_hand_acc = pd.read_csv("acc_F7:68:55:8D:84:0E.csv")
df_n_right_hand_gyro = pd.read_csv("gyro_F7:68:55:8D:84:0E.csv")


df_q_chest = pd.read_csv("quat_F8:DC:C7:F1:48:7A.csv")
df_q_left_hand = pd.read_csv("quat_E6:4F:B9:D7:18:7C.csv")
df_q_right_knee = pd.read_csv("quat_F0:3D:E7:ED:F6:F7.csv")


#---------------------------------------------------------------------
df_n_chest_acc = df_n_chest_acc.sort_values(by='host_time')
df_n_chest_gyro = df_n_chest_gyro.sort_values(by='host_time')

df_n_left_knee_acc = df_n_left_knee_acc.sort_values(by='host_time')
df_n_left_knee_gyro = df_n_left_knee_gyro.sort_values(by='host_time')

df_n_right_hand_acc = df_n_right_hand_acc.sort_values(by='host_time')  
df_n_right_hand_gyro = df_n_right_hand_gyro.sort_values(by='host_time')

df_q_chest = df_q_chest.sort_values(by='host_time')
df_q_left_hand = df_q_left_hand.sort_values(by='host_time')
df_q_right_knee = df_q_right_knee.sort_values(by='host_time')

#---------------------------------------------------------------------
# Sincronizar los datos usando merge_asof()



df_sync = pd.merge_asof(df_n_chest_acc, df_n_chest_gyro, on='host_time')
df_sync = pd.merge_asof(df_sync, df_n_left_knee_acc, on='host_time')
df_sync = pd.merge_asof(df_sync, df_n_left_knee_gyro, on='host_time')
df_sync = pd.merge_asof(df_sync, df_n_right_hand_acc, on='host_time')
df_sync = pd.merge_asof(df_sync, df_n_right_hand_gyro, on='host_time')
df_sync = pd.merge_asof(df_sync, df_q_chest, on='host_time')
df_sync = pd.merge_asof(df_sync, df_q_left_hand, on='host_time')
df_sync = pd.merge_asof(df_sync, df_q_right_knee, on='host_time')


# Guardar el resultado en un nuevo archivo CSV

df_sync = df_sync[desired_cols]
df_sync.to_csv("Synchronized_Sensor_Data.csv", index=False)

# Mostrar las primeras filas del dataset sincronizado
print(df_sync.head())


