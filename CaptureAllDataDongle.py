from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import FnVoid_VoidP_VoidP
from time import sleep
import platform
import sys
import signal
import time, datetime
import csv, os
import subprocess

# Variables globales para los datos de sensores
sensor_data = []
states = []  # Aquí se almacenan las instancias de State

# Definir direcciones MAC de sensores y dongles directamente en el código
sensor_addresses = [
    # "EE:1B:72:FA:BF:E8"
    # "F1:1E:E2:6F:1D:E1",
    # "CE:94:48:FE:5D:C5"
    "F7:68:55:8D:84:0E"
    # "F9:8C:1E:4A:F5:D0",
    # "FA:F1:20:99:CB:B4"
]

dongles = [
    # "00:E0:5C:48:06:BD",
    # "00:E0:5C:48:01:34",
    # "00:E0:5C:48:03:93"
    "3C:0A:F3:10:17:F0"
]

def force_disconnect_sensors():
    print("Escaneando y desconectando sensores en todos los dongles...")

    try:
        # Verificar qué dongles están disponibles
        result = subprocess.run(["hcitool", "dev"], capture_output=True, text=True)
        dongles = [line.split()[1] for line in result.stdout.splitlines() if "hci" in line]

        if not dongles:
            print("No se detectaron dongles Bluetooth. Verifica que estén conectados.")
            return
        
        print(f"Dongles detectados: {dongles}")

        # Obtener todos los dispositivos conectados
        result = subprocess.run(["hcitool", "con"], capture_output=True, text=True)
        connections = result.stdout.splitlines()

        for line in connections:
            if "handle" in line:
                parts = line.split()
                mac_address = parts[2]  # Extraer MAC Address del sensor
                print(f"Desconectando {mac_address} en todos los dongles...")

                # Intentar desconectar el dispositivo en cada dongle
                for dongle in dongles:
                    subprocess.run(["bluetoothctl", "disconnect", mac_address], capture_output=True, text=True)
                    subprocess.run(["bluetoothctl", "remove", mac_address], capture_output=True, text=True)

        # Habilitar Bluetooth en caso de que estuviera bloqueado
        subprocess.run(["rfkill", "unblock", "bluetooth"])
        print("Todos los sensores han sido desconectados correctamente.")
        sleep(2)

    except Exception as e:
        print(f"Error al desconectar sensores: {e}")

# Definicion del manejador ISR
def handler_timer(signum, frame):
    # Aqui se van a guardar los datos que contiene el vector de 14 posiciones en un arreglo
    for state in states:
        latest_data = state.get_latest_data()
        state.samples += 1
        
        # Verificar que no haya datos vacíos (None) en la lectura actual
        if None not in latest_data['quaternion']:
            # Nombre del archivo CSV basado en la dirección MAC
            file_name = f"sensor_data_{state.device.address}.csv"

            # Verificar si el archivo existe, si no, escribir el encabezado
            file_exists = os.path.isfile(file_name)

            # Escribir datos en el archivo CSV
            with open(file_name, mode='a', newline='') as file:
                writer = csv.writer(file)
                
                # Si el archivo no existe, escribimos los encabezados
                if not file_exists:
                    writer.writerow([
                        # 'time',
                        'time',
                        'timestamp', 
                        'quat_w', 'quat_x', 'quat_y', 'quat_z', 
                    ])
                
                # Obtener el tiempo actual en formato HH:MM:SS
                # current_time = datetime.datetime.now().strftime('%H:%M:%S')
                current_time2 = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-4]  # Usamos [: -4] para truncar a dos dígitos en milisegundos

                # Escribir los datos reales
                writer.writerow([
                    # current_time,
                    current_time2,
                    latest_data['timestamp'], 
                    *latest_data['quaternion'], 

                ])

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.latest_data = [None] * 5  # 11 posiciones: timestamp + quaternion + acc + gyro
        self.quaternion_callback = FnVoid_VoidP_DataP(self.quaternion_handler)
        # self.acc_callback = FnVoid_VoidP_DataP(self.acc_handler)
        # self.gyro_callback = FnVoid_VoidP_DataP(self.gyro_handler)
        # self.mag_callback = FnVoid_VoidP_DataP(self.mag_handler)
    
    def quaternion_handler(self, ctx, data):
        quaternion = parse_value(data)
        timestamp = time.time()
        self.latest_data[0] = timestamp
        self.latest_data[1:5] = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        # self.samples += 1

    # def acc_handler(self, ctx, data):
    #     acc = parse_value(data)
    #     self.latest_data[5:8] = [acc.x, acc.y, acc.z]

    # def gyro_handler(self, ctx, data):
    #     gyro = parse_value(data)
    #     self.latest_data[8:11] = [gyro.x, gyro.y, gyro.z]

    # def mag_handler(self, ctx, data):
    #     mag = parse_value(data)
    #     self.latest_data[11:14] = [mag.x, mag.y, mag.z]

    def get_latest_data(self):
        # Devolver los datos más recientes de timestamp, quaternion, acc, gyro, y mag
        return{
            'timestamp': self.latest_data[0],
            'quaternion': self.latest_data[1:5],
            # 'acc': self.latest_data[5:8],
            # 'gyro': self.latest_data[8:11]
            # 'mag': self.latest_data[11:14]
        }

def assign_sensors_to_dongles(sensor_addresses, dongles):
    """
    Asigna sensores a dongles de manera equitativa, máximo 2 sensores por dongle.
    """
    dongle_assignments = {dongle: [] for dongle in dongles}
    
    for i, sensor in enumerate(sensor_addresses):
        dongle = dongles[i % len(dongles)]  # Asignación circular
        dongle_assignments[dongle].append(sensor)
    
    return dongle_assignments

def connect_sensors(sensor_addresses, dongles, max_retries=5):
    global states
    dongle_assignments = assign_sensors_to_dongles(sensor_addresses, dongles)
    
    for dongle, sensors in dongle_assignments.items():
        print(f"Using dongle {dongle} for sensors: {sensors}")
        for address in sensors:
            connected = False
            for attempt in range(max_retries):
                try:
                    d = MetaWear(address, hci_mac=dongle)  # Conexión específica al dongle
                    d.connect()
                    if d.is_connected:
                        print(f"Connected to {d.address} via dongle {dongle}")
                        state = State(d)
                        states.append(state)
                        connected = True
                        break
                    else:
                        print(f"Failed to connect to {d.address} via {dongle}")
                except Exception as e:
                    print(f"Connection attempt {attempt + 1} to {address} via {dongle} failed: {e}")
                sleep(2)  # Espera antes de reintentar
            if not connected:
                print(f"Could not connect to sensor {address} after {max_retries} attempts.")
                sys.exit(1)  # Salir si algún sensor no se conecta
    return states

def configure_and_subscribe_sensors(states):
    for state in states:
        d = state.device
        print("Configuring device " + d.address)

        libmetawear.mbl_mw_settings_set_connection_parameters(d.board, 30.0, 50.0, 0, 4000)
        sleep(1.5)

        # Configuración de Sensor Fusion
        libmetawear.mbl_mw_sensor_fusion_set_mode(d.board, SensorFusionMode.IMU_PLUS)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(d.board, SensorFusionAccRange._8G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(d.board, SensorFusionGyroRange._2000DPS)
        libmetawear.mbl_mw_sensor_fusion_write_config(d.board)

        # Suscripción a quaternion
        signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_subscribe(signal_quat, None, state.quaternion_callback)

        # # Suscripción a acelerómetro
        # signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(d.board)
        # libmetawear.mbl_mw_datasignal_subscribe(signal_acc, None, state.acc_callback)

        # # Suscripción a giroscopio
        # signal_gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(d.board)
        # libmetawear.mbl_mw_datasignal_subscribe(signal_gyro, None, state.gyro_callback)

        # # Suscripción a magnetómetro
        # signal_mag = libmetawear.mbl_mw_mag_bmm150_get_b_field_data_signal(d.board)
        # libmetawear.mbl_mw_datasignal_subscribe(signal_mag, None, state.mag_callback)

        # Habilitar y comenzar a obtener datos de todos los sensores
        libmetawear.mbl_mw_sensor_fusion_enable_data(d.board, SensorFusionData.QUATERNION)
        # libmetawear.mbl_mw_acc_enable_acceleration_sampling(d.board)
        # libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(d.board)
        # libmetawear.mbl_mw_mag_bmm150_enable_b_field_sampling(d.board)

        libmetawear.mbl_mw_sensor_fusion_start(d.board)
        # libmetawear.mbl_mw_acc_start(d.board)
        # libmetawear.mbl_mw_gyro_bmi160_start(d.board)
        # libmetawear.mbl_mw_mag_bmm150_start(d.board)
    
    return states

def disconnect_sensors(states):
    for state in states:
        print("Disconnecting device " + state.device.address)

        # Stop signals
        libmetawear.mbl_mw_sensor_fusion_stop(state.device.board)
        # libmetawear.mbl_mw_acc_stop(state.device.board)
        # libmetawear.mbl_mw_gyro_bmi160_stop(state.device.board)
        # libmetawear.mbl_mw_mag_bmm150_stop(state.device.board)

        # Disable signals
        # libmetawear.mbl_mw_acc_disable_acceleration_sampling(state.device.board)
        # libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(state.device.board)
        # libmetawear.mbl_mw_mag_bmm150_disable_b_field_sampling(state.device.board)

        # Unsubscribe signals
        signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(state.device.board, SensorFusionData.QUATERNION)
        # signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(state.device.board)
        # signal_gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(state.device.board)
        # signal_mag = libmetawear.mbl_mw_mag_bmm150_get_b_field_data_signal(state.device.board)

        libmetawear.mbl_mw_datasignal_unsubscribe(signal_quat)
        # libmetawear.mbl_mw_datasignal_unsubscribe(signal_acc)
        # libmetawear.mbl_mw_datasignal_unsubscribe(signal_gyro)
        # libmetawear.mbl_mw_datasignal_unsubscribe(signal_mag)

        libmetawear.mbl_mw_debug_disconnect(state.device.board)
        print("Disconnected from " + state.device.address)

        sleep(1)

    print("Total Samples Received")
    for state in states:
        print("%s -> %d" % (state.device.address, state.samples))

def on_disconnect(ctx, board):
    print("Lost connection, attempting to reconnect…")
    # tear down your state for this device, then:
    board.connect(board.address, board.hci_mac)


def main():
    force_disconnect_sensors()
    states = connect_sensors(sensor_addresses, dongles)
    configure_and_subscribe_sensors(states)

    # Configuracion del manejador ISR
    signal.signal(signal.SIGALRM, handler_timer)
    signal.setitimer(signal.ITIMER_REAL, 0.02, 0.02)
    
    def signal_handler(sig, frame):
        signal.setitimer(signal.ITIMER_REAL, 0, 0)
        print("\nCtrl+C detected, disconnecting sensors...")
        disconnect_sensors(states)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print("Streaming data... Press Ctrl+C to stop.")
    while True:
        time.sleep(5)  # Mantener viva la ejecución del hilo principal

if __name__ == "__main__":
    main()