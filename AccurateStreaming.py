from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, datetime, os, csv, signal, sys

# Sensor y dongle MACs
device_macs = ["C4:65:87:1A:13:0B","D5:42:DD:AC:BE:E1", "E6:AC:5E:B8:4C:D9", "F0:3D:E7:ED:F6:F7", "E6:4F:B9:D7:18:7C", "CE:5A:39:E6:8F:B3"]
# dongle_macs = ["3C:0A:F3:10:17:F0"]
dongle_macs = ["90:DE:80:E1:8F:B0","00:E0:5C:48:03:93","00:E0:5C:48:01:63"]
states = []

# Asegura desconexión previa

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
                    subprocess.run(["bluetoothctl", "disconnect", mac], capture_output=True)
                    subprocess.run(["bluetoothctl", "remove", mac], capture_output=True)
        subprocess.run(["rfkill", "unblock", "bluetooth"])
        time.sleep(2)
    except Exception:
        pass

# Clase de estado para escribir CSV directamente en callbacks

class State:
    def __init__(self, device):
        self.device = device
        base_dir = "DriveUpload"
        os.makedirs(base_dir, exist_ok=True)
        self.acc_file  = os.path.join(base_dir, f"acc_{device.address}.csv")
        self.gyro_file = os.path.join(base_dir, f"gyro_{device.address}.csv")
        # Headers: full-precision human timestamps k
        if not os.path.isfile(self.gyro_file):
            with open(self.gyro_file, 'w', newline='') as f:
                csv.writer(f).writerow([
                    'host_time', 'sensor_time',
                    'gyro_x', 'gyro_y', 'gyro_z'
                ])
        # Callbacks
        self.acc_cb  = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_cb = FnVoid_VoidP_DataP(self.gyro_data_handler)

    def acc_data_handler(self, ctx, data_ptr):
        # Full-precision sensor timestamp (converted to human-readable)
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        # Full-precision host timestamp
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        with open(self.acc_file, 'a', newline='') as f:
            csv.writer(f).writerow([
                host_time, sensor_time,
                val.x, val.y, val.z
            ])
    def gyro_data_handler(self, ctx, data_ptr):
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        with open(self.gyro_file, 'a', newline='') as f:
            csv.writer(f).writerow([
                host_time, sensor_time,
                val.x, val.y, val.z
            ])

    def get_acc_cb(self):
        return self.acc_cb

    def get_gyro_cb(self):
        return self.gyro_cb
# Asigna sensores a dongles en modo circular

def assign_sensors_to_dongles(devices, dongles):
    assign = {d:[] for d in dongles}
    for i, mac in enumerate(devices):
        assign[dongles[i % len(dongles)]].append(mac)
    return assign

# Conecta sensores y retorna instancias State

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
                    print(f"Conn err {mac}: {e}")
                    time.sleep(1)
    return states

# Configura y suscribe sensores por separado
def configure_and_subscribe_sensors(states):
    for st in states:
        b = st.device.board
        # ACC
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
        # GYRO
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)
        # Subscribe ACC
        sig_a = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
        libmetawear.mbl_mw_datasignal_subscribe(sig_a, None, st.get_acc_cb())
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(b)
        libmetawear.mbl_mw_acc_start(b)
        # Subscribe GYRO
        sig_g = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
        libmetawear.mbl_mw_datasignal_subscribe(sig_g, None, st.get_gyro_cb())
        libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(b)
        libmetawear.mbl_mw_gyro_bmi270_start(b)

# Desconexión limpia
def disconnect_sensors():
    for st in states:
        b = st.device.board
        libmetawear.mbl_mw_acc_stop(b)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(b)
        libmetawear.mbl_mw_gyro_bmi270_stop(b)
        libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(b)
        # unsubscribe
        sig_a = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
        sig_g = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
        libmetawear.mbl_mw_datasignal_unsubscribe(sig_a)
        libmetawear.mbl_mw_datasignal_unsubscribe(sig_g)
        libmetawear.mbl_mw_debug_disconnect(b)
        time.sleep(1)
    print("All disconnected")

# Main loop
if __name__ == '__main__':
    force_disconnect_sensors()
    connect_sensors(device_macs, dongle_macs)
    configure_and_subscribe_sensors(states)

    def on_exit(sig, frame):
        print("\nStopping...")
        disconnect_sensors()
        sys.exit(0)
    signal.signal(signal.SIGINT, on_exit)
    print("Streaming 50 Hz to separate acc_/gyro_ CSVs...")
    while True:
        time.sleep(1)
