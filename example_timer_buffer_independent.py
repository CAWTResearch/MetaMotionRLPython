from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from time import sleep
from threading import Lock
from collections import deque
import sys

class State:
    def __init__(self, device, buffer_lock, combined_buffer, buffer_index):
        self.device = device
        self.latest_data = None
        self.buffer_lock = buffer_lock
        self.combined_buffer = combined_buffer
        self.buffer_index = buffer_index
        self.callback = FnVoid_VoidP_DataP(self.data_handler)

    def data_handler(self, ctx, data):
        # Parsear el valor recibido
        parsed_data = parse_value(data)
        self.latest_data = [parsed_data.w, parsed_data.x, parsed_data.y, parsed_data.z]
        print(f"Sensor: {self.device.address} {self.latest_data}")

        # Actualizar el buffer combinado con bloqueo
        with self.buffer_lock:
            self.combined_buffer[self.buffer_index] = self.latest_data

        # Imprimir el buffer combinado
        self.print_combined_buffer()

    def print_combined_buffer(self):
        with self.buffer_lock:
            print("Combined Buffer:")
            print(self.combined_buffer)

def connect_and_configure_sensors(device_addresses):
    buffer_lock = Lock()
    combined_buffer = [[None, None, None, None] for _ in range(len(device_addresses))]  # Buffer combinado
    states = []

    for i, address in enumerate(device_addresses):
        d = MetaWear(address)
        d.connect()
        print("Connected to " + d.address + " over " + ("USB" if d.usb.is_connected else "BLE"))
        state = State(d, buffer_lock, combined_buffer, i)
        states.append(state)

    for s in states:
        print("Configuring device")
        libmetawear.mbl_mw_settings_set_connection_parameters(s.device.board, 7.5, 7.5, 0, 6000)
        sleep(1.5)
        libmetawear.mbl_mw_sensor_fusion_set_mode(s.device.board, SensorFusionMode.NDOF)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(s.device.board, SensorFusionAccRange._8G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(s.device.board, SensorFusionGyroRange._2000DPS)
        libmetawear.mbl_mw_sensor_fusion_write_config(s.device.board)
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_subscribe(signal, None, s.callback)
        libmetawear.mbl_mw_sensor_fusion_enable_data(s.device.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_sensor_fusion_start(s.device.board)

    return states

def disconnect_sensors(states):
    for s in states:
        libmetawear.mbl_mw_sensor_fusion_stop(s.device.board)
        signal = libmetawear.mbl_mw_sensor_fusion_get_data_signal(s.device.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_unsubscribe(signal)
        libmetawear.mbl_mw_debug_disconnect(s.device.board)
        print("Disconnected from " + s.device.address)

def main():
    device_addresses = sys.argv[1:]
    states = connect_and_configure_sensors(device_addresses)
    try:
        print("Streaming data. Press CTRL+C to stop.")
        while True:
            sleep(1)  # Mantén el script en ejecución para recibir datos
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    finally:
        disconnect_sensors(states)
        print("All sensors disconnected.")

if __name__ == "__main__":
    main()
