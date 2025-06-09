# usage: python3 log_acc_multiple.py
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
import threading, time, csv, os, sys, termios, tty, subprocess
from threading import Event

# -----------------------------------------------------------------------------
# 1) GLOBAL CONFIG
# -----------------------------------------------------------------------------
sensor_addresses = [
    "F8:DC:C7:F1:48:7A", "FC:97:E9:E0:E8:E4", "F7:68:55:8D:84:0E", "E6:4F:B9:D7:18:7C"
]
# sensor_addresses = [
#     "F4:73:A1:AB:BB:64"
# ]
dongles = [
    "00:E0:5C:48:0B:98", "00:E0:5C:48:01:34", "00:E0:5C:48:02:38", "90:DE:80:E1:8F:B0", "3C:0A:F3:10:17:F0"
]
# Interval (seconds) between periodic download calls
DOWNLOAD_INTERVAL = 0.5

# -----------------------------------------------------------------------------
# 2) FORCE‐DISCONNECT ANY EXISTING CONNECTIONS
# -----------------------------------------------------------------------------
def force_disconnect_sensors():
    print("Escaneando y desconectando sensores en todos los dongles...")
    try:
        result = subprocess.run(["hcitool", "dev"], capture_output=True, text=True)
        dongles_list = [line.split()[1] for line in result.stdout.splitlines() if "hci" in line]
        if not dongles_list:
            print("No se detectaron dongles Bluetooth. Verifica que estén conectados.")
            return

        print(f"Dongles detectados: {dongles_list}")
        result = subprocess.run(["hcitool", "con"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if "handle" in line:
                parts = line.split()
                mac_address = parts[2]
                print(f"Desconectando {mac_address} en todos los dongles...")
                for dongle in dongles_list:
                    subprocess.run(["bluetoothctl", "disconnect", mac_address], capture_output=True, text=True)
                    subprocess.run(["bluetoothctl", "remove", mac_address], capture_output=True, text=True)

        subprocess.run(["rfkill", "unblock", "bluetooth"])
        print("Todos los sensores han sido desconectados correctamente.")
        sleep(2)

    except Exception as e:
        print(f"Error al desconectar sensores: {e}")

# -----------------------------------------------------------------------------
# 3) STATE CLASS: holds per‐sensor logger & CSV handles + stop flags
# -----------------------------------------------------------------------------
class State:
    def __init__(self, device):
        self.device = device
        self.board = device.board

        # We'll track the two “logger” objects returned by create_voidp(...)
        self.acc_logger = None
        self.gyro_logger = None

        # Threads & events for periodic download
        self.download_thread = None
        self.stop_download = Event()
        self.download_done = Event()

        # Open CSV files (one for accel, one for gyro)
        base_dir = "downloadTrial"
        os.makedirs(base_dir, exist_ok=True)
        mac_clean = device.address
        self.acc_path  = os.path.join(base_dir, f"acc_{mac_clean}.csv")
        self.gyro_path = os.path.join(base_dir, f"gyro_{mac_clean}.csv")
        os.makedirs(os.path.dirname(self.acc_path) or ".", exist_ok=True)

        # Open in “append” mode
        self.acc_f  = open(self.acc_path,  "a", newline="")
        self.gyro_f = open(self.gyro_path, "a", newline="")
        self.acc_w  = csv.writer(self.acc_f)
        self.gyro_w = csv.writer(self.gyro_f)

        # If new file, write header
        if os.stat(self.acc_path).st_size == 0:
            self.acc_w.writerow(["host_time", "sensor_time", "acc_x", "acc_y", "acc_z"])
        if os.stat(self.gyro_path).st_size == 0:
            self.gyro_w.writerow(["host_time", "sensor_time", "gyro_x", "gyro_y", "gyro_z"])

        # Prepare callbacks for logger‐subscribe
        self.cb_acc  = FnVoid_VoidP_DataP(self.on_acc_download)
        self.cb_gyro = FnVoid_VoidP_DataP(self.on_gyro_download)

    def on_acc_download(self, ctx, entry):
        # Don’t write if the file is already closed:
        if self.acc_f.closed:
            return
        val = parse_value(entry)
        sensor_time = datetime.fromtimestamp(entry.contents.epoch / 1000.0) \
                             .strftime('%H:%M:%S.%f')
        host_time = datetime.now().strftime('%H:%M:%S.%f')
        self.acc_w.writerow([host_time, sensor_time, val.x, val.y, val.z])
        self.acc_f.flush()

    def on_gyro_download(self, ctx, entry):
        # Don’t write if the file is already closed:
        if self.gyro_f.closed:
            return
        val = parse_value(entry)
        sensor_time = datetime.fromtimestamp(entry.contents.epoch / 1000.0) \
                             .strftime('%H:%M:%S.%f')
        host_time = datetime.now().strftime('%H:%M:%S.%f')
        self.gyro_w.writerow([host_time, sensor_time, val.x, val.y, val.z])
        self.gyro_f.flush()

    def start_loggers(self):
        b = self.board

        # --- Configure accelerometer ---
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)

        # --- Configure gyroscope ---
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)

        # Retrieve data signals
        sig_acc  = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
        sig_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)

        # Create “logger” for each
        self.acc_logger = create_voidp(lambda cb: 
            libmetawear.mbl_mw_datasignal_log(sig_acc, None, cb),
            resource="acc_logger"
        )
        self.gyro_logger = create_voidp(lambda cb:
            libmetawear.mbl_mw_datasignal_log(sig_gyro, None, cb),
            resource="gyro_logger"
        )

        # Subscribe callbacks (will be invoked during download)
        libmetawear.mbl_mw_logger_subscribe(self.acc_logger, None, self.cb_acc)
        libmetawear.mbl_mw_logger_subscribe(self.gyro_logger, None, self.cb_gyro)

        # Finally start flash‐logging on the board
        libmetawear.mbl_mw_logging_start(b, 0)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)
        libmetawear.mbl_mw_acc_start(b)
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)
        libmetawear.mbl_mw_gyro_bmi270_start(b)

        print(f"[{self.device.address}] Logging started.")

    def _download_loop(self):
        # Build a single LogDownloadHandler instance
        def prog(ctx, left, total):
            if left == 0:
                self.download_done.set()

        fn_prog = FnVoid_VoidP_UInt_UInt(prog)
        handler = LogDownloadHandler(
            context=None,
            received_progress_update=fn_prog,
            received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP)
        )

        while not self.stop_download.is_set():
            # Clear event and ask for “all available pages” (0 offset)
            self.download_done.clear()
            libmetawear.mbl_mw_logging_download(self.board, 0, byref(handler))
            # Wait until this download chunk finishes (or timeout)
            self.download_done.wait(timeout=DOWNLOAD_INTERVAL + 1)
            time.sleep(DOWNLOAD_INTERVAL)

    def start_downloader(self):
        self.download_thread = threading.Thread(
            target=self._download_loop, daemon=True
        )
        self.download_thread.start()

class State:
    def __init__(self, device):
        self.device = device
        self.board = device.board

        # Two “logger” objects returned by create_voidp(...)
        self.acc_logger = None
        self.gyro_logger = None

        # Threads & events for periodic download
        self.download_thread = None
        self.stop_download = Event()
        self.download_done = Event()

        # Create folder "downloadTrial" and open CSVs there
        base_dir = "downloadTrial"
        os.makedirs(base_dir, exist_ok=True)
        mac_clean = device.address.replace(":", "")
        self.acc_path  = os.path.join(base_dir, f"acc_{mac_clean}.csv")
        self.gyro_path = os.path.join(base_dir, f"gyro_{mac_clean}.csv")

        self.acc_f  = open(self.acc_path,  "a", newline="")
        self.gyro_f = open(self.gyro_path, "a", newline="")
        self.acc_w  = csv.writer(self.acc_f)
        self.gyro_w = csv.writer(self.gyro_f)

        # If new, write header
        if os.stat(self.acc_path).st_size == 0:
            self.acc_w.writerow(["host_time", "sensor_time", "acc_x", "acc_y", "acc_z"])
        if os.stat(self.gyro_path).st_size == 0:
            self.gyro_w.writerow(["host_time", "sensor_time", "gyro_x", "gyro_y", "gyro_z"])

        # Prepare callbacks for logger‐subscribe
        self.cb_acc  = FnVoid_VoidP_DataP(self.on_acc_download)
        self.cb_gyro = FnVoid_VoidP_DataP(self.on_gyro_download)

    def on_acc_download(self, ctx, entry):
        if self.acc_f.closed:
            return
        val = parse_value(entry)
        sensor_time = datetime.fromtimestamp(entry.contents.epoch / 1000.0)\
                           .strftime('%H:%M:%S.%f')
        host_time = datetime.now().strftime('%H:%M:%S.%f')
        self.acc_w.writerow([host_time, sensor_time, val.x, val.y, val.z])
        self.acc_f.flush()

    def on_gyro_download(self, ctx, entry):
        if self.gyro_f.closed:
            return
        val = parse_value(entry)
        sensor_time = datetime.fromtimestamp(entry.contents.epoch / 1000.0)\
                           .strftime('%H:%M:%S.%f')
        host_time = datetime.now().strftime('%H:%M:%S.%f')
        self.gyro_w.writerow([host_time, sensor_time, val.x, val.y, val.z])
        self.gyro_f.flush()

    def start_loggers(self):
        b = self.board
        time.sleep(0.3)  # allow GATT to settle after connect

        # --- Configure accelerometer ---
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)

        # --- Configure gyroscope ---
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)

        # Retrieve data signals
        sig_acc  = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
        sig_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)

        # Create “logger” for each
        self.acc_logger = create_voidp(lambda cb:
            libmetawear.mbl_mw_datasignal_log(sig_acc, None, cb),
            resource="acc_logger"
        )
        self.gyro_logger = create_voidp(lambda cb:
            libmetawear.mbl_mw_datasignal_log(sig_gyro, None, cb),
            resource="gyro_logger"
        )

        # Subscribe callbacks
        libmetawear.mbl_mw_logger_subscribe(self.acc_logger, None, self.cb_acc)
        libmetawear.mbl_mw_logger_subscribe(self.gyro_logger, None, self.cb_gyro)

        # Start flash‐logging & sampling
        libmetawear.mbl_mw_logging_start(b, 0)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)
        libmetawear.mbl_mw_acc_start(b)
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)
        libmetawear.mbl_mw_gyro_bmi270_start(b)

        print(f"[{self.device.address}] Logging started.")

    def _download_loop(self):
        def prog(ctx, left, total):
            if left == 0:
                self.download_done.set()

        fn_prog = FnVoid_VoidP_UInt_UInt(prog)
        handler = LogDownloadHandler(
            context=None,
            received_progress_update=fn_prog,
            received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP)
        )

        while not self.stop_download.is_set():
            self.download_done.clear()
            libmetawear.mbl_mw_logging_download(self.board, 0, byref(handler))
            self.download_done.wait(timeout=DOWNLOAD_INTERVAL + 1)
            time.sleep(DOWNLOAD_INTERVAL)

    def start_downloader(self):
        self.download_thread = threading.Thread(
            target=self._download_loop, daemon=True
        )
        self.download_thread.start()

    def final_download(self,
                       per_try_timeout: float = 3.0,
                       max_retries: int   = 5):
        """
        Repeatedly call mbl_mw_logging_download(...) until we see left==0
        (self.download_done.set()) or until we exceed max_retries attempts
        with no “left==0” callback. Each attempt waits up to per_try_timeout
        seconds for left==0 before retrying. If we hit max_retries with
        no progress, we break out (warning).
        """
        def prog(ctx, left, total):
            if left == 0:
                self.download_done.set()

        fn_prog = FnVoid_VoidP_UInt_UInt(prog)
        handler = LogDownloadHandler(
            context=None,
            received_progress_update=fn_prog,
            received_unknown_entry=cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP)
        )

        retries = 0
        while True:
            # Clear the event and request “all pages (offset=0)”
            self.download_done.clear()
            libmetawear.mbl_mw_logging_download(self.board, 0, byref(handler))

            # Wait up to per_try_timeout for left == 0
            got_all = self.download_done.wait(timeout=per_try_timeout)
            if got_all:
                # We got left==0 for this batch. Now do one more quick check
                # to see if any pages popped in while we were waiting.
                self.download_done.clear()
                libmetawear.mbl_mw_logging_download(self.board, 0, byref(handler))
                if self.download_done.wait(timeout=per_try_timeout):
                    # left==0 again → truly empty. We’re done.
                    print(f"[{self.device.address}] final_download: all pages received.")
                    break
                else:
                    # New pages arrived; try again from top.
                    retries = 0
                    continue
            else:
                # We never saw left==0 within per_try_timeout.
                retries += 1
                if retries >= max_retries:
                    print(f"[{self.device.address}] final_download: giving up after "
                          f"{max_retries} tries (no left==0).")
                    break
                else:
                    # Wait a moment, then retry
                    time.sleep(0.2)
                    continue



    def stop_and_cleanup(self):
        b = self.board

        # Stop flash logging
        libmetawear.mbl_mw_logging_stop(b)
        libmetawear.mbl_mw_logging_flush_page(b)
        libmetawear.mbl_mw_logging_clear_entries(b)

        # Stop sampling
        libmetawear.mbl_mw_acc_stop(b)
        libmetawear.mbl_mw_gyro_bmi270_stop(b)

        # Close CSV handles
        self.acc_f.close()
        self.gyro_f.close()

        print(f"[{self.device.address}] Stopped logging & downloaded all data.")

    def disconnect(self):
        # Finally disconnect over BLE
        libmetawear.mbl_mw_debug_disconnect(self.board)
        print(f"[{self.device.address}] Disconnected.")

# -----------------------------------------------------------------------------
# 4) CONNECT ALL SENSORS
# -----------------------------------------------------------------------------
def assign_sensors_to_dongles(sensor_addresses, dongles):
    dongle_assignments = {dongle: [] for dongle in dongles}
    for i, sensor in enumerate(sensor_addresses):
        dongle = dongles[i % len(dongles)]
        dongle_assignments[dongle].append(sensor)
    return dongle_assignments

def connect_sensors(sensor_addresses, dongles, max_retries=5):
    states = []
    dongle_assignments = assign_sensors_to_dongles(sensor_addresses, dongles)
    for dongle, sensors in dongle_assignments.items():
        print(f"[+] Using dongle {dongle} for sensors: {sensors}")
        for address in sensors:
            for attempt in range(max_retries):
                try:
                    dev = MetaWear(address, hci_mac=dongle)
                    dev.connect()
                    if dev.is_connected:
                        print(f"[+] Connected to {dev.address} via {dongle}")
                        st = State(dev)
                        states.append(st)
                        break
                    else:
                        print(f"[-] Failed to connect to {address} on {dongle}")
                except Exception as e:
                    print(f"    Attempt {attempt+1} → {address} failed: {e}")
                sleep(2)
            else:
                print(f"[!] Could not connect to sensor {address} after {max_retries} attempts")
                sys.exit(1)
    return states

# -----------------------------------------------------------------------------
# 5) WAIT FOR KEYPRESS (shared among all sensors)
# -----------------------------------------------------------------------------
def wait_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# -----------------------------------------------------------------------------
# 6) MAIN: configure everything, start logging & downloading in parallel
# -----------------------------------------------------------------------------
def main():
    force_disconnect_sensors()
    states = connect_sensors(sensor_addresses, dongles)
    # for st in states:
    #     print("Debug reset")
    #     libmetawear.mbl_mw_debug_reset(st.device.board)
    #     time.sleep(2.0)
    #     print("debugged")
    # 1) For each sensor, start flash‐logging + spawn its downloader thread
    for st in states:
        libmetawear.mbl_mw_logging_clear_entries(st.board)
        st.start_loggers()
        st.start_downloader()

    print("\nLogging + downloading on all sensors. Press any key to stop…")
    # wait_key()
    time.sleep(180.0)  # Blocks until user presses a key
    print("Logging has Stopped")

    # 2) Signal all downloader threads to stop, then join them
    for st in states:
        st.stop_download.set()
    for st in states:
        if st.download_thread is not None:
            st.download_thread.join()
    for st in states:
        st.final_download(
            per_try_timeout=3.0,  # wait up to 3s each attempt
            max_retries=5         # try at most 5 times
        )
    # 3) Stop & clean up each sensor (stop logging, unsubscribe, close CSVs)
    for st in states:
        st.stop_and_cleanup()

    # 4) Finally disconnect each sensor
    for st in states:
        print("Debug reset")
        libmetawear.mbl_mw_debug_reset(st.device.board)
        time.sleep(2.0)
        print("debugged")

    for st in states:
        st.disconnect()
        sleep(1)

    print("\nAll sensors processed. Exiting.")

if __name__ == "__main__":
    main()
