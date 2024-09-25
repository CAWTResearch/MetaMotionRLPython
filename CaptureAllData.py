from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from time import sleep
import platform
import sys
import signal
import time

# Variables globales para los datos de sensores
sensor_data = []
states = []  # Aquí se almacenan las instancias de State

# Definicion del manejador ISR
def handler_timer(signum, frame):
    # Aqui se van a guardar los datos que contiene el vector de 14 posiciones en un arreglo
    for state in states:
        latest_data = state.get_latest_data()
        print(f"Datos más recientes del sensor {state.device.address}: {latest_data}") #

# Configuracion del manejador ISR
signal.signal(signal.SIGALRM, handler_timer)
signal.setitimer(signal.ITIMER_REAL, 0.02, 0.02)

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.latest_data = [None] * 14  # 14 posiciones: timestamp + quaternion + acc + gyro + mag
        self.quaternion_callback = FnVoid_VoidP_DataP(self.quaternion_handler)
        self.acc_callback = FnVoid_VoidP_DataP(self.acc_handler)
        self.gyro_callback = FnVoid_VoidP_DataP(self.gyro_handler)
        self.mag_callback = FnVoid_VoidP_DataP(self.mag_handler)
    
    def quaternion_handler(self, ctx, data):
        quaternion = parse_value(data)
        timestamp = time.time()
        self.latest_data[0] = timestamp
        self.latest_data[1:5] = [quaternion.w, quaternion.x, quaternion.y, quaternion.z]
        self.samples += 1

    def acc_handler(self, ctx, data):
        acc = parse_value(data)
        self.latest_data[5:8] = [acc.x, acc.y, acc.z]

    def gyro_handler(self, ctx, data):
        gyro = parse_value(data)
        self.latest_data[8:11] = [gyro.x, gyro.y, gyro.z]

    def mag_handler(self, ctx, data):
        mag = parse_value(data)
        self.latest_data[11:14] = [mag.x, mag.y, mag.z]

    def get_latest_data(self):
        # Devolver los datos más recientes de timestamp, quaternion, acc, gyro, y mag
        return{
            'timestamp': self.latest_data[0],
            'quaternion': self.latest_data[1:5],
            'acc': self.latest_data[5:8],
            'gyro': self.latest_data[8:11],
            'mag': self.latest_data[11:14]
        }

def connect_sensors(sensor_addresses):
    global states
    for address in sensor_addresses:
        d = MetaWear(address)
        d.connect()
        if d.is_connected:
            print("Connected to " + d.address + " over " + ("USB" if d.usb.is_connected else "BLE"))
            state = State(d)
            states.append(state)
        else:
            print("Failed to connect to " + d.address)
            sys.exit(1)  # Exit if any sensor fails to connect
    
    return states

def configure_and_subscribe_sensors(states):
    for state in states:
        d = state.device
        print("Configuring device " + d.address)

        libmetawear.mbl_mw_settings_set_connection_parameters(d.board, 7.5, 7.5, 0, 6000)
        sleep(1.5)

        # Configuración de Sensor Fusion
        libmetawear.mbl_mw_sensor_fusion_set_mode(d.board, SensorFusionMode.NDOF)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(d.board, SensorFusionAccRange._8G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(d.board, SensorFusionGyroRange._2000DPS)
        libmetawear.mbl_mw_sensor_fusion_write_config(d.board)

        # Suscripción a quaternion
        signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_subscribe(signal_quat, None, state.quaternion_callback)

        # Suscripción a acelerómetro
        signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(d.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal_acc, None, state.acc_callback)

        # Suscripción a giroscopio
        signal_gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(d.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal_gyro, None, state.gyro_callback)

        # Suscripción a magnetómetro
        signal_mag = libmetawear.mbl_mw_mag_bmm150_get_b_field_data_signal(d.board)
        libmetawear.mbl_mw_datasignal_subscribe(signal_mag, None, state.mag_callback)

        # Habilitar y comenzar a obtener datos de todos los sensores
        libmetawear.mbl_mw_sensor_fusion_enable_data(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(d.board)
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(d.board)
        libmetawear.mbl_mw_mag_bmm150_enable_b_field_sampling(d.board)

        libmetawear.mbl_mw_sensor_fusion_start(d.board)
        libmetawear.mbl_mw_acc_start(d.board)
        libmetawear.mbl_mw_gyro_bmi160_start(d.board)
        libmetawear.mbl_mw_mag_bmm150_start(d.board)
    
    return states

def disconnect_sensors(states):
    for state in states:
        print("Disconnecting device " + state.device.address)

        # Stop signals
        libmetawear.mbl_mw_sensor_fusion_stop(state.device.board)
        libmetawear.mbl_mw_acc_stop(state.device.board)
        libmetawear.mbl_mw_gyro_bmi160_stop(state.device.board)
        libmetawear.mbl_mw_mag_bmm150_stop(state.device.board)

        # Disable signals
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(state.device.board)
        libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(state.device.board)
        libmetawear.mbl_mw_mag_bmm150_disable_b_field_sampling(state.device.board)

        # Unsubscribe signals
        signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(state.device.board, SensorFusionData.QUATERNION)
        signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(state.device.board)
        signal_gyro = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(state.device.board)
        signal_mag = libmetawear.mbl_mw_mag_bmm150_get_b_field_data_signal(state.device.board)

        libmetawear.mbl_mw_datasignal_unsubscribe(signal_quat)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal_acc)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal_gyro)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal_mag)

        libmetawear.mbl_mw_debug_disconnect(state.device.board)
        print("Disconnected from " + state.device.address)

    print("Total Samples Received")
    for state in states:
        print("%s -> %d" % (state.device.address, state.samples))

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 stream_sensors.py [mac1] [mac2] ... [mac(n)]")
        sys.exit(1)

    sensor_addresses = sys.argv[1:]

    # Paso 1: Conectar sensores
    states = connect_sensors(sensor_addresses)

    # Paso 2: Configurar y suscribir solo si todos los sensores se conectaron correctamente
    configure_and_subscribe_sensors(states)
    
    def signal_handler(sig, frame):
        print("\nCtrl+C detected, disconnecting sensors...")
        disconnect_sensors(states)
        sys.exit(0)

    signal.signal(signal.SIGINT, signal_handler)

    print("Streaming data... Press Ctrl+C to stop.")
    while True:
        time.sleep(5)  # Mantener viva la ejecución del hilo principal

if __name__ == "__main__":
    main()
