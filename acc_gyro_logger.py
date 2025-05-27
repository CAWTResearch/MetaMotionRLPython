# usage: python3 log_acc.py [mac]
from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp, create_voidp_int
from mbientlab.metawear.cbindings import *
from time import sleep
from mbientlab.metawear.cbindings import (
    LogDownloadHandler,
    FnVoid_VoidP_DataP,
    FnVoid_VoidP_UInt_UInt,
    FnVoid_VoidP_VoidP_VoidP_UInt,
    FnVoid_VoidP_UByte_Long_UByteP_UByte
)
from ctypes import cast, POINTER, c_void_p, byref
import platform
import signal
import time, threading
from datetime import datetime

from threading  import Event
import csv, os
import subprocess

import sys

# Variables globales para los datos de sensores
acc_data = []
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

class State:
    def __init__(self, device):
        self.device = device
        self.samples = 0
        self.latest_data = [None] * 6  # 11 posiciones: timestamp + quaternion + acc + gyro
        self.acc_callback = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.logger = None
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

def configure_and_log_sensors(states):
    for state in states:
        d = state.device
        board= d.board
        print("Configuring device " + d.address)
        print("Get and log acc signal")

        libmetawear.mbl_mw_settings_set_connection_parameters(d.board, 30.0, 50.0, 0, 4000)
        sleep(1.5)

        libmetawear.mbl_mw_acc_bmi270_set_odr(board, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(board, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(board)

        libmetawear.mbl_mw_gyro_bmi270_set_odr( board, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range( board, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(board)

        signal_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(d.board)
        signal_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(d.board) 
        acc_logger = gyro_logger = None
        try: 
            acc_logger = create_voidp(
                lambda cb: libmetawear.mbl_mw_datasignal_log(signal_acc, None, cb), resource = "acc_logger"    
            )

            gyro_logger = create_voidp(
                lambda cb: libmetawear.mbl_mw_datasignal_log(signal_gyro, None, cb), resource = "gyro_logger"            
            )
        except RuntimeError as err:
            print(err)
            return

        state.logger = [acc_logger, gyro_logger]


        libmetawear.mbl_mw_logging_start(board, 0)


        print("Start logging")
        libmetawear.mbl_mw_logging_start(d.board, 0)
        
        print("Start acc")
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(d.board)
        libmetawear.mbl_mw_acc_start(d.board)

        print("start gyro")
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(d.board)
        libmetawear.mbl_mw_gyro_bmi270_start(d.board)
        
        print("Logging data for 10s")
        sleep(5.0)
    
        print("Stop logging")
        libmetawear.mbl_mw_logging_stop(d.board)

        print("Flush cache if MMS")
        libmetawear.mbl_mw_logging_flush_page(d.board)
        print("Flushed")

        # Prepare two CSVs
        mac_clean = d.address.replace(":", "")
        acc_csv = open(f"acc_{mac_clean}.csv", "a", newline="")
        gyro_csv = open(f"gyro_{mac_clean}.csv", "a", newline="")
        acc_writer = csv.writer(acc_csv)
        gyro_writer = csv.writer(gyro_csv)

        # write headers if new
        if os.stat(acc_csv.name).st_size == 0:
            acc_writer.writerow(["epoch_ms","acc_x","acc_y","acc_z"])
        if os.stat(gyro_csv.name).st_size == 0:
            gyro_writer.writerow(["epoch_ms","gyro_x","gyro_y","gyro_z"])

        # progress sync
        e = Event()
        def progress_update(context, left, total):
            if left == 0:
                e.set()
        fn_prog = FnVoid_VoidP_UInt_UInt(progress_update)

        download_handler = LogDownloadHandler(
            context=None,
            received_progress_update=fn_prog,
            received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP)
        )

        # callbacks to write each entry
        def acc_download(ctx, entry):
            val = parse_value(entry)
            epoch = entry.contents.epoch
            ts = datetime.fromtimestamp(epoch / 1000.0) \
                .strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]  
            acc_writer.writerow([ts, val.x, val.y, val.z])

        def gyro_download(ctx, entry):
            val = parse_value(entry)
            epoch = entry.contents.epoch
            ts       = datetime.fromtimestamp(epoch / 1000.0) \
                    .strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
            gyro_writer.writerow([ts, val.x, val.y, val.z])

        cb_acc = FnVoid_VoidP_DataP(acc_download)
        cb_gyro = FnVoid_VoidP_DataP(gyro_download)

        # subscribe both
        libmetawear.mbl_mw_logger_subscribe(acc_logger, None, cb_acc)
        libmetawear.mbl_mw_logger_subscribe(gyro_logger, None, cb_gyro)

        # download everything
        libmetawear.mbl_mw_logging_download(d.board, 0, byref(download_handler))
        e.wait()

        # close files
        acc_csv.close()
        gyro_csv.close()


        # # Prepare CSV file for this device
        # csv_filename = f"acc_gyro_{d.address.replace(':','')}.csv"
        # file_exists = os.path.isfile(csv_filename)
        # csv_file = open(csv_filename, 'a', newline='')
        # csv_writer = csv.writer(csv_file)
        # if not file_exists:
        #     csv_writer.writerow([
        #         'epoch_ms', 'acc_x', 'acc_y', 'acc_z', 'gyro_x', 'gyro_y', 'gyro_z'
        #     ])

        # e = Event()

        # print("CSV prepared")

        # def progress_update_handler(context, entries_left, total_entries):
        #     if (entries_left == 0):
        #         e.set()

        # fn_wrapper = FnVoid_VoidP_UInt_UInt(progress_update_handler)
        
        # # Create a download handler that calls our progress callback
        # download_handler = LogDownloadHandler(
        #     context = None,
        #     received_progress_update = fn_wrapper,
        #     # we won’t use unknown or unhandled entry callbacks
        #     received_unknown_entry = cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
        #     received_unhandled_entry = cast(None, FnVoid_VoidP_DataP)

        # )

        # # This callback writes each sample to CSV
        # def download_data_handler(ctx, entry):
        #     val = parse_value(entry)
        #     epoch = entry.contents.epoch
        #     # assuming an accel signal log: val.x, val.y, val.z
        #     csv_writer.writerow([epoch, val.x, val.y, val.z, val.x, val.y, val.z])
        # callback = FnVoid_VoidP_DataP(download_data_handler)

        # # subscribe our writer-callback to the logger
        # libmetawear.mbl_mw_logger_subscribe(state.logger, None, callback)
        # # kick off the download process (the '0' is for default settings)
        # libmetawear.mbl_mw_logging_download(d.board, 0, byref(download_handler))
        # e.wait()

        # csv_file.close()
    
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
    configure_and_log_sensors(states)
    print("Resetting device")
    
    e = Event()
    for state in states:
        state.on_disconnect = lambda status: e.set()
        print("Debug reset")
        libmetawear.mbl_mw_debug_reset(state.device.board)
        e.wait()
        print("debugged")

    disconnect_sensors(states)

if __name__ == "__main__":
    main()