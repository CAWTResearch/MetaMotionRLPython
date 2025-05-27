from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import (
    LogDownloadHandler,
    FnVoid_VoidP_DataP,
    FnVoid_VoidP_UInt_UInt,
    FnVoid_VoidP_VoidP_VoidP_UInt,
    FnVoid_VoidP_UByte_Long_UByteP_UByte
)
from time import sleep
from ctypes import cast, POINTER, c_void_p, byref
import platform
import sys
import signal
import time, datetime, threading
from threading  import Event
import csv, os
import subprocess

# Variables globales para los datos de sensores
acc_gyro_data = []
states = []  # Aquí se almacenan las instancias de State

# Definir direcciones MAC de sensores y dongles directamente en el código
sensor_addresses = [
    # "F7:68:55:8D:84:0E"
    "D5:42:DD:AC:BE:E1"
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
        if None not in latest_data['acc']:
            # Nombre del archivo CSV basado en la dirección MAC
            file_name = f"acc_gyro_{state.device.address}.csv"

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
                        'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z', 
                    ])
                
                # Obtener el tiempo actual en formato HH:MM:SS
                current_time2 = datetime.datetime.now().strftime('%H:%M:%S.%f')[:-4]  # Usamos [: -4] para truncar a dos dígitos en milisegundos

                # Escribir los datos reales
                writer.writerow([
                    # current_time,
                    current_time2,
                    latest_data['timestamp'], 
                    *latest_data['acc'], 
                    *latest_data['gyro'] 
                ])
    
class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.latest_data = [None] * 6  # 11 posiciones: timestamp + quaternion + acc + gyro
        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_callback = FnVoid_VoidP_DataP(self.gyro_data_handler)
        # self.mag_callback = FnVoid_VoidP_DataP(self.mag_handler)

    # acc callback
    def acc_data_handler(self, ctx, data):
        print("ACC: %s -> %s" % (self.device.address, parse_value(data)))
        acc = parse_value(data)
        self.latest_data[1:4] = [acc.x, acc.y, acc.z]
                
    # gyro callback
    def gyro_data_handler(self, ctx, data):
        print("GYRO: %s -> %s" % (self.device.address, parse_value(data)))
        gyro = parse_value(data)
        self.latest_data[4:7] = [gyro.x, gyro.y, gyro.z]

    def get_latest_data(self):
        # Devolver los datos más recientes de timestamp, quaternion, acc, gyro, y mag
        return{
            'timestamp': self.latest_data[0],
            'acc': self.latest_data[1:4],
            'gyro': self.latest_data[4:7]
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

        # Setup acc
        libmetawear.mbl_mw_acc_bmi270_set_odr(d.board, AccBmi270Odr._50Hz) # BMI 270 specific call
        libmetawear.mbl_mw_acc_bosch_set_range(d.board, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(d.board)

        # signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(d.board)
        # logger = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(signal, None, fn), resource = "acc_logger")

        # setup gyro
        libmetawear.mbl_mw_gyro_bmi270_set_range(d.board, GyroBoschRange._1000dps);
        libmetawear.mbl_mw_gyro_bmi270_set_odr(d.board, GyroBoschOdr._50Hz);
        libmetawear.mbl_mw_gyro_bmi270_write_config(d.board);

        # Suscripción a acelerómetro
        signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(d.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal_acc, None, state.acc_callback)
        libmetawed = state.device
        # Habilitar y comenzar a obtener datos de todos los sensores
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(d.board)
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(d.board)
        signal_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(d.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal_gyro, None, state.gyro_callback)

        libmetawear.mbl_mw_acc_start(d.board)
        libmetawear.mbl_mw_gyro_bmi270_start(d.board)
    
    return states

def disconnect_sensors(states):
    for state in states:
        print("Disconnecting device " + state.device.address)

        # Stop signals
        libmetawear.mbl_mw_acc_stop(state.device.board)
        libmetawear.mbl_mw_gyro_bmi270_stop(state.device.board)

        # Disable signals
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(state.device.board)
        libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(state.device.board)

        # Unsubscribe signals
        signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(state.device.board)
        signal_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(state.device.board)

        libmetawear.mbl_mw_datasignal_unsubscribe(signal_acc)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal_gyro)

        libmetawear.mbl_mw_debug_disconnect(state.device.board)
        print("Disconnected from " + state.device.address)

        sleep(1)

    print("Total Samples Received")
    for state in states:
        print("%s -> %d" % (state.device.address, state.samples))

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