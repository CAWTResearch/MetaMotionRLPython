# usage: python3 log_acc.py [mac]
from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp
from mbientlab.metawear.cbindings import *
from time import sleep
from mbientlab.metawear.cbindings import (
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange,
    LogDownloadHandler,
    FnVoid_VoidP_DataP,
    FnVoid_VoidP_UInt_UInt,
    FnVoid_VoidP_UByte_Long_UByteP_UByte
)
from ctypes import cast, byref
from datetime import datetime
import threading, time, csv, os, sys, termios, tty
from threading  import Event
import subprocess

# Variables globales para los datos de sensores
acc_data = []
states = []  # Aquí se almacenan las instancias de State

# Definir direcciones MAC de sensores y dongles directamente en el código
sensor_addresses = [
    # "F7:68:55:8D:84:0E"
    "D5:42:DD:AC:BE:E1"
]

dongles = [
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

def wait_key():
    """Block until any key is pressed."""
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def configure_and_log_with_periodic_download(state, download_interval=2.0):
    d     = state.device
    board = d.board

    # --- 1) sensor config & logger setup ---
    libmetawear.mbl_mw_acc_bmi270_set_odr(board, AccBmi270Odr._50Hz)
    libmetawear.mbl_mw_acc_bosch_set_range(board, AccBoschRange._4G)
    libmetawear.mbl_mw_acc_write_acceleration_config(board)

    libmetawear.mbl_mw_gyro_bmi270_set_odr(board, GyroBoschOdr._50Hz)
    libmetawear.mbl_mw_gyro_bmi270_set_range(board, GyroBoschRange._1000dps)
    libmetawear.mbl_mw_gyro_bmi270_write_config(board)

    sig_acc  = libmetawear.mbl_mw_acc_get_acceleration_data_signal(board)
    sig_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(board)

    acc_logger = create_voidp(lambda cb:
        libmetawear.mbl_mw_datasignal_log(sig_acc, None, cb),
        resource="acc_logger"
    )
    gyro_logger = create_voidp(lambda cb:
        libmetawear.mbl_mw_datasignal_log(sig_gyro, None, cb),
        resource="gyro_logger"
    )
    state.logger = (acc_logger, gyro_logger)

    libmetawear.mbl_mw_logging_start(board, 0)
    libmetawear.mbl_mw_acc_enable_acceleration_sampling(board)
    libmetawear.mbl_mw_acc_start(board)
    libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(board)
    libmetawear.mbl_mw_gyro_bmi270_start(board)

    # --- 2) open CSVs & subscribe download callbacks ---
    mac_clean = d.address.replace(":", "")
    acc_f  = open(f"acc_{mac_clean}.csv",  "a", newline="")
    gyro_f = open(f"gyro_{mac_clean}.csv","a", newline="")
    acc_w  = csv.writer(acc_f)
    gyro_w = csv.writer(gyro_f)
    if os.stat(acc_f.name).st_size == 0:
        acc_w.writerow(["timestamp","acc_x","acc_y","acc_z"])
    if os.stat(gyro_f.name).st_size == 0:
        gyro_w.writerow(["timestamp","gyro_x","gyro_y","gyro_z"])

    def on_acc_download(ctx, entry):
        val = parse_value(entry)
        ts  = datetime.now().isoformat(timespec="milliseconds")
        acc_w.writerow([ts, val.x, val.y, val.z])
        acc_f.flush()

    def on_gyro_download(ctx, entry):
        val = parse_value(entry)
        ts  = datetime.now().isoformat(timespec="milliseconds")
        gyro_w.writerow([ts, val.x, val.y, val.z])
        gyro_f.flush()

    cb_acc = FnVoid_VoidP_DataP(on_acc_download)
    cb_gyro= FnVoid_VoidP_DataP(on_gyro_download)
    libmetawear.mbl_mw_logger_subscribe(acc_logger, None, cb_acc)
    libmetawear.mbl_mw_logger_subscribe(gyro_logger, None, cb_gyro)

    # --- 3) set up download handler & progress event ---
    download_done = threading.Event()
    def prog(ctx, left, total):
        if left == 0:
            download_done.set()
    fn_prog = FnVoid_VoidP_UInt_UInt(prog)
    handler = LogDownloadHandler(
        context=None,
        received_progress_update=fn_prog,
        received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
        received_unhandled_entry=cast(None, FnVoid_VoidP_DataP)
    )

    # --- 4) downloader thread ---
    stop_download = threading.Event()
    def downloader_loop():
        while not stop_download.is_set():
            download_done.clear()
            libmetawear.mbl_mw_logging_download(board, 0, byref(handler))
            # wait until this chunk is done (or timeout)
            download_done.wait(timeout=download_interval + 1)
            time.sleep(download_interval)

    t = threading.Thread(target=downloader_loop, daemon=True)
    t.start()

    print("Logging & downloading in background — press any key to stop")
    wait_key()
    stop_download.set()
    t.join()

    # --- 5) clean up logger & CSVs ---
    libmetawear.mbl_mw_logging_stop(board)
    libmetawear.mbl_mw_logging_flush_page(board)
    libmetawear.mbl_mw_logging_clear_entries(board)
    libmetawear.mbl_mw_acc_stop(board)
    libmetawear.mbl_mw_gyro_bmi270_stop(board)
    libmetawear.mbl_mw_datasignal_unsubscribe(state.logger[0])
    libmetawear.mbl_mw_datasignal_unsubscribe(state.logger[1])
    acc_f.close()
    gyro_f.close()

    print("Finished logging + downloading for", d.address)


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
    for st in states:
        configure_and_log_with_periodic_download(st, download_interval=2.0)
        libmetawear.mbl_mw_debug_reset_after_gc(st.device.board)
        sleep(1.0)

    disconnect_sensors(states)

if __name__ == "__main__":
    main()