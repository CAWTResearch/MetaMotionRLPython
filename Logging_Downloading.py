from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    LogDownloadHandler,
    FnVoid_VoidP_UInt_UInt,
    FnVoid_VoidP_UByte_Long_UByteP_UByte,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, datetime, os, csv, signal, sys
from threading import Event
from ctypes import byref, cast

# ------------------------------------------------------------
# 1) CONFIG: list of sensor MACs and dongle MACs, plus duration
# ------------------------------------------------------------
device_macs = [
    "C4:65:87:1A:13:0B",
    "D5:42:DD:AC:BE:E1",
    "F0:3D:E7:ED:F6:F7",
    "CE:5A:39:E6:8F:B3",
    "E6:AC:5E:B8:4C:D9"
]
dongle_macs = [
    "00:E0:5C:48:01:63",
    "D8:3A:DD:EA:0C:EF",
    "00:E0:5C:48:06:BD"
]
# How long (in seconds) to let each sensor log into flash
LOG_DURATION = 60

# A global list of State instances (one per sensor)
states = []

# ------------------------------------------------------------
# 2) FORCE‐DISCONNECT UTILITY (unchanged)
# ------------------------------------------------------------
def force_disconnect_sensors():
    try:
        result = subprocess.run(["hcitool", "dev"], capture_output=True, text=True)
        dlist = [l.split()[1] for l in result.stdout.splitlines() if "hci" in l]
        if not dlist:
            return
        result = subprocess.run(["hcitool", "con"], capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if "handle" in line:
                mac = line.split()[2]
                for d in dlist:
                    subprocess.run(
                        ["bluetoothctl", "disconnect", mac],
                        capture_output=True
                    )
                    subprocess.run(
                        ["bluetoothctl", "remove", mac],
                        capture_output=True
                    )
        subprocess.run(["rfkill", "unblock", "bluetooth"])
        time.sleep(2)
    except Exception:
        pass

# ------------------------------------------------------------
# 3) STATE CLASS: holds per‐sensor logger objects & in‐memory buffers
# ------------------------------------------------------------
class State:
    def __init__(self, device):
        self.device = device
        self.board = device.board

        # Directory where CSVs will be written
        base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
        os.makedirs(base_dir, exist_ok=True)

        # Use MAC (no colons) as part of the filename
        mac_no_colon = device.address

        self.acc_file  = os.path.join(base_dir, f"acc_{mac_no_colon}.csv")
        self.gyro_file = os.path.join(base_dir, f"gyro_{mac_no_colon}.csv")

        # In‐memory buffers for “downloaded” rows
        self.acc_data_list  = []
        self.gyro_data_list = []

        # These will be set once we create each logger below
        self.acc_logger  = None
        self.gyro_logger = None

        # Wrap the callbacks
        self.acc_cb  = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_cb = FnVoid_VoidP_DataP(self.gyro_data_handler)

    def acc_data_handler(self, ctx, data_ptr):
        """
        Called during the DOWNLOAD phase—whenever a logged
        accelerometer packet is received. “data_ptr.contents.epoch”
        is in ms since epoch.
        """
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z
        self.acc_data_list.append((host_time, sensor_time, x, y, z))

    def gyro_data_handler(self, ctx, data_ptr):
        """
        Called during the DOWNLOAD phase—whenever a logged
        gyroscope packet is received.
        """
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z
        self.gyro_data_list.append((host_time, sensor_time, x, y, z))

    def dump_to_csv(self):
        """
        Once all logging + download is finished, write the in‐memory
        lists out to CSV files.
        """
        # ACC
        with open(self.acc_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(['host_time', 'sensor_time', 'acc_x', 'acc_y', 'acc_z'])
            writer.writerows(self.acc_data_list)

        # GYRO
        with open(self.gyro_file, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(['host_time', 'sensor_time', 'gyro_x', 'gyro_y', 'gyro_z'])
            writer.writerows(self.gyro_data_list)

# ------------------------------------------------------------
# 4) HELPER: assign sensors to dongles in round‐robin
# ------------------------------------------------------------
def assign_sensors_to_dongles(devices, dongles):
    assign = {d: [] for d in dongles}
    for i, mac in enumerate(devices):
        assign[dongles[i % len(dongles)]].append(mac)
    return assign

# ------------------------------------------------------------
# 5) CONNECT ALL SENSORS
# ------------------------------------------------------------
def connect_sensors(devices, dongles, retries=5):
    for dongle, devs in assign_sensors_to_dongles(devices, dongles).items():
        for mac in devs:
            for _ in range(retries):
                try:
                    m = MetaWear(mac, hci_mac=dongle)
                    m.connect()
                    if m.is_connected:
                        print(f"Connected {mac} via {dongle}")
                        st = State(m)
                        states.append(st)
                        break
                except Exception as e:
                    print(f"[WARN] Connection error for {mac}: {e}")
                    time.sleep(1)
    return states

# ------------------------------------------------------------
# 6) CONFIGURE + START LOGGING FOR EACH SENSOR
# ------------------------------------------------------------
def configure_and_start_logging(states):
    for st in states:
        b = st.board

        # --- ACCELEROMETER CONFIG ---
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)

        # Create an ACC “logger” on that signal:
        sig_acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
        logger_acc = libmetawear.mbl_mw_logger_create(sig_acc)
        st.acc_logger = logger_acc

        # Subscribe the logger so that when we DOWNLOAD later, our acc_data_handler() runs:
        libmetawear.mbl_mw_logger_subscribe(logger_acc, None, st.acc_cb)

        # Optional: specify how many samples to save before a flash‐page is committed.
        # (If you omit this, firmware uses its own default page size.)
        # libmetawear.mbl_mw_logger_set_save_count(logger_acc, 100)

        # --- GYROSCOPE CONFIG ---
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)

        sig_gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
        logger_gyro = libmetawear.mbl_mw_logger_create(sig_gyro)
        st.gyro_logger = logger_gyro
        libmetawear.mbl_mw_logger_subscribe(logger_gyro, None, st.gyro_cb)
        # libmetawear.mbl_mw_logger_set_save_count(logger_gyro, 100)

        # --- FINALLY: START LOGGING ON THE BOARD ---
        # (This tells the MetaWear firmware: “Begin sampling & saving each
        #  packet in flash via those loggers.”)
        libmetawear.mbl_mw_logging_start(b)

        print(f"Started logging on {st.device.address}")

# ------------------------------------------------------------
# 7) STOP LOGGING (so no more samples are pushed into flash)
# ------------------------------------------------------------
def stop_logging(states):
    for st in states:
        b = st.board
        libmetawear.mbl_mw_logging_stop(b)
        print(f"Stopped logging on {st.device.address}")

# ------------------------------------------------------------
# 8) DOWNLOAD ALL LOGGED DATA, IN‐ORDER
# ------------------------------------------------------------
def download_all(states):
    """
    For each State, issue a mbl_mw_logging_download call. We use one shared
    Event per-sensor to block until “left == 0” in progress callback.
    """
    for st in states:
        b = st.board
        e = Event()

        # Progress‐update callback: when ‘left == 0’, the download is done
        def progress_update_handler(ctx, left, total):
            if left == 0:
                e.set()

        # You can detect unknown entry‐IDs (if something weird was logged)
        def unknown_entry_handler(ctx, id, epoch, data, length):
            print(
                f"[WARN] Sensor {b.address} sent an unknown log entry id={id} "
                f"at epoch {epoch}"
            )

        progress_fn = FnVoid_VoidP_UInt_UInt(progress_update_handler)
        unknown_fn  = FnVoid_VoidP_UByte_Long_UByteP_UByte(unknown_entry_handler)
        download_handler = LogDownloadHandler(
            context=None,
            received_progress_update=progress_fn,
            received_unknown_entry=unknown_fn,
            received_unhandled_entry=cast(None, FnVoid_VoidP_DataP),
        )

        # Kick off download of “everything” (0 == download from oldest page to newest)
        libmetawear.mbl_mw_logging_download(b, 0, byref(download_handler))
        print(f"Downloading logs from {b.address} …")
        e.wait()
        print(f"Download complete for {b.address}.")

# ------------------------------------------------------------
# 9) DISCONNECT CLEANLY (and erase flash so sensor is ready next time)
# ------------------------------------------------------------
def disconnect_sensors():
    for st in states:
        b = st.board

        # 1) Stop whichever sampling is still running
        libmetawear.mbl_mw_acc_stop(b)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(b)

        libmetawear.mbl_mw_gyro_bmi270_stop(b)
        libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(b)

        # 2) Unsubscribe logger callbacks (no more log notifications)
        if st.acc_logger is not None:
            libmetawear.mbl_mw_logger_unsubscribe(st.acc_logger)
        if st.gyro_logger is not None:
            libmetawear.mbl_mw_logger_unsubscribe(st.gyro_logger)

    # 3) Finally, reset & disconnect each board
    for st in states:
        b = st.board
        print(f"Reset & disconnecting {st.device.address} …")
        libmetawear.mbl_mw_debug_reset(b)
        time.sleep(1)
        libmetawear.mbl_mw_debug_disconnect(b)
        time.sleep(0.5)

    print("All sensors disconnected.")

# ------------------------------------------------------------
# 10) MAIN: coordinate logging → wait → download → dump → exit
# ------------------------------------------------------------
if __name__ == '__main__':
    # a) Force‐kick any existing BLE connections
    force_disconnect_sensors()

    # b) Connect all sensors into ‘states’ list
    connect_sensors(device_macs, dongle_macs)

    # c) Configure each sensor and start flash‐logging
    configure_and_start_logging(states)

    # d) Let them log for LOG_DURATION seconds (in flash)
    print(f"Logging to flash for {LOG_DURATION} seconds …")
    start_ts = time.time()
    try:
        while time.time() - start_ts < LOG_DURATION:
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\nInterrupted by user → stopping early.")
    finally:
        # e) Always stop logging (so the firmware finalizes pages)
        stop_logging(states)

    # f) Download every logged packet over BLE
    download_all(states)

    # g) Disconnect cleanup
    disconnect_sensors()

    # h) Write CSV files to disk
    for st in states:
        st.dump_to_csv()
        print(f"Wrote {len(st.acc_data_list)} accel rows → {st.acc_file}")
        print(f"Wrote {len(st.gyro_data_list)} gyro rows → {st.gyro_file}")

    print("All done. Exiting.")
    sys.exit(0)
