from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP, FnVoid_VoidP, 
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, datetime, os, csv, signal, sys, threading, glob

# Sensor y dongle MACs
# device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "E6:AC:5E:B8:4C:D9",'F8:DC:C7:F1:48:7A',"E6:4F:B9:D7:18:7C"]
# device_macs = ['F8:DC:C7:F1:48:7A',"E6:4F:B9:D7:18:7C"]
device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "D5:42:DD:AC:BE:E1", "E6:4F:B9:D7:18:7C" ,"E6:AC:5E:B8:4C:D9"]
# device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "E6:AC:5E:B8:4C:D9"]
dongle_macs = ['00:E0:5C:48:00:2F','00:E0:5C:48:01:63', 'D8:3A:DD:EA:0C:EF', '00:E0:5C:48:03:93', '00:E0:5C:48:06:BD']
# dongle_macs = ['D8:3A:DD:EA:0C:EF', '00:E0:5C:48:03:93', '00:E0:5C:48:06:BD']
# dongle_macs = ["00:E0:5C:48:01:70","00:E0:5C:48:02:38", "00:E0:5C:48:01:34", "00:E0:5C:48:0B:98", "00:E0:5C:48:00:DA"]

states = []
        # hci5    00:E0:5C:48:01:70
        # hci1    00:E0:5C:48:00:DA
        # hci4    00:E0:5C:48:0B:98
        # hci3    00:E0:5C:48:01:34
        # hci2    3C:0A:F3:10:17:F0
        # hci0    00:E0:5C:48:02:38

profiles = [
    {"interval":25.0, "latency":2, "timeout":20000, "offset":0},
    {"interval":30.0, "latency":3, "timeout":20000, "offset":10},
    {"interval":35.0, "latency":4, "timeout":20000, "offset":20},
    {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
    {"interval":7.5, "latency":0, "timeout":20000, "offset":30},
]
# profiles = [
#     {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
#     {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
#     {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
#     {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
#     {"interval":40.0, "latency":5, "timeout":20000, "offset":30},
# ]


# Asegura desconexión previa
STREAM_DURATION = 30

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

        self.acc_count  = 0
        self.gyro_count = 0

        # Base folder for final CSVs; ensure it exists
        base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
        os.makedirs(base_dir, exist_ok=True)

        # Remember each sensor's MAC (without colons) to name files
        mac_no_colon = device.address

        # Full paths where we’ll dump at the end:
        self.acc_file  = os.path.join(base_dir, f"acc_{mac_no_colon}.csv")
        self.gyro_file = os.path.join(base_dir, f"gyro_{mac_no_colon}.csv")


        # ACC file
        self._acc_fh = open(self.acc_file,  "a", newline='')
        self._acc_writer = csv.writer(self._acc_fh)
        # write header only if file is new/empty
        if os.path.getsize(self.acc_file) == 0:
            self._acc_writer.writerow(['host_time','sensor_time','acc_x','acc_y','acc_z'])

        # GYRO file
        self._gyro_fh = open(self.gyro_file, "a", newline='')
        self._gyro_writer = csv.writer(self._gyro_fh)
        if os.path.getsize(self.gyro_file) == 0:
            self._gyro_writer.writerow(['host_time','sensor_time','gyro_x','gyro_y','gyro_z'])
        
        # Prepare callback wrappers
        self.acc_cb  = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_cb = FnVoid_VoidP_DataP(self.gyro_data_handler)


    def acc_data_handler(self, ctx, data_ptr):
        # Called on each accelerometer sample
        # 1) sensor timestamp → human‐readable
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        # 2) host timestamp
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        # 3) parse x,y,z
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z

        self._acc_writer.writerow([host_time, sensor_time, x, y, z])
        self._acc_fh.flush()
        self.acc_count += 1

    def gyro_data_handler(self, ctx, data_ptr):
        # Same as above, but for gyroscope
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z

        self._gyro_writer.writerow([host_time, sensor_time, x, y, z])
        self._gyro_fh.flush()
        self.gyro_count += 1

    def get_acc_cb(self):
        return self.acc_cb

    def get_gyro_cb(self):
        return self.gyro_cb

    def close_files(self):
        self._acc_fh.close()
        self._gyro_fh.close()

def assign_sensors_to_dongles(devices, dongles):
    assign = {d:[] for d in dongles}
    for i, mac in enumerate(devices):
        assign[dongles[i % len(dongles)]].append(mac)
    return assign

# Conecta sensores y retorna instancias State

def connect_sensors(devices, dongles, retries=10):
    for dongle, devs in assign_sensors_to_dongles(devices, dongles).items():
        for mac in devs:
            for _ in range(retries):
                try:
                    m = MetaWear(mac, hci_mac=dongle)
                    m.connect()
                    if m.is_connected:
                        print(f"Connected {mac} via {dongle}")
                        st = State(m)
                        st.profile = profiles[len(states)]
                        m.on_disconnect = lambda status, st=st: reconfigure_and_subscribe(st)
                        time.sleep(0.1)
                        states.append(st)
                        break
                except Exception as e:
                    print(f"Conn err {mac}: {e}")
                    time.sleep(1)
    return states

# Configura y suscribe sensores por separado
def configure_and_subscribe_sensors(states):
    for st, profile in zip(states, profiles):
        b = st.device.board


        libmetawear.mbl_mw_settings_set_connection_parameters(
            b,
            profile["interval"],   # min & max the same
            profile["interval"],
            profile["latency"],
            profile["timeout"]
        )
        time.sleep(1.5)
        libmetawear.mbl_mw_settings_set_tx_power(b, 8)
        time.sleep(1.5)

        # ACC: set ODR and range
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
 

        # GYRO: set ODR and range
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)  # Give time for config to apply
    
    for st in states:
        b = st.device.board
        

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
# Replace your old disconnect_sensors() with this:

def disconnect_sensors():
    for st in states:
        b = st.device.board

        # 1) Stop accel sampling
        libmetawear.mbl_mw_acc_stop(b)

        libmetawear.mbl_mw_acc_disable_acceleration_sampling(b)


        # 2) Stop gyro sampling
        libmetawear.mbl_mw_gyro_bmi270_stop(b)
   

        libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(b)
  
        # 3) Unsubscribe from accel signal
        acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)

        libmetawear.mbl_mw_datasignal_unsubscribe(acc_signal)
     

        # 4) Unsubscribe from gyro signal
        gyro_signal = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
    
        libmetawear.mbl_mw_datasignal_unsubscribe(gyro_signal)
        
    for st in states:

        print("Debug")
        # libmetawear.mbl_mw_debug_reset(st.device.board)
        time.sleep(2.0)
        # print("debugged")

        # 5) Finally, disconnect over BLE
        libmetawear.mbl_mw_debug_disconnect(b)

        # Give the board a moment to process each step
        time.sleep(1.0)

    print("All disconnected")


def reconfigure_and_subscribe(st, retries=5, backoff=1.0):
    # BlueZ will call this on disconnect; immediately spin off a thread
    threading.Thread(target=_do_reconnect, args=(st, retries, backoff), daemon=True).start()

def _do_reconnect(st, retries, backoff):
    dev     = st.device
    b       = dev.board
    mac     = dev.address
    profile = st.profile

    print(f"[WARN] Lost connection to {mac}.  Performing full reset…")

    # 1) One big board‐side reset clears out all streams & subscriptions
    if dev.is_connected:
        print("No disconnect, I lied!")
        return
    try:
        dev.disconnect()
    except Exception:
        pass

    timeout = time.time() + 5.0
    while dev.is_connected and time.time() < timeout:
        time.sleep(0.05)
    if dev.is_connected:
        print(f"  • warning: disconnect() did not finish in time")


    # 3) Give BlueZ another moment
    time.sleep(backoff)

    # 4) Retry connect() up to `retries` times
    for i in range(1, retries+1):
        try:
            print("It reconnected Yahid!")
            dev.connect()
            if dev.is_connected:
                print(f"[OK] Reconnected to {mac} on try #{i}")
                break
        except Exception as e:
            print(f"  • connect #{i} failed: {e}")
        time.sleep(backoff)
    else:
        print(f"[ERROR] Could not reconnect to {mac} after {retries} tries")
        return




# Main loop
if __name__ == '__main__':

    base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
    for old in glob.glob(os.path.join(base_dir, "*.csv")):
        try:
            os.remove(old)
            print(f"Deleted old file {old}")
        except OSError as e:
            print(f"Could not delete {old}: {e}")

    force_disconnect_sensors()
    connect_sensors(device_macs, dongle_macs)
    configure_and_subscribe_sensors(states)

    # d) Allow Ctrl+C to abort early
    def on_exit(sig, frame):
        print("\nInterrupted by user!")
        disconnect_sensors()
        for st in states:
            st.close_files()
            print(f"  • Wrote {st.acc_count} accel rows → {st.acc_file}")
            print(f"  • Wrote {st.gyro_count} gyro rows → {st.gyro_file}")
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    # e) Timer loop
    print(f"Streaming 50 Hz from each sensor for {STREAM_DURATION} seconds…")
    start_ts = time.time()
    try:
        while True:
            elapsedtime= time.time()-start_ts
            if elapsedtime >= STREAM_DURATION:
                print(f"\n{STREAM_DURATION} seconds elapsed. Stopping streaming…")
                break
            time.sleep(0.1)
            print(f"{elapsedtime}")
        
    except KeyboardInterrupt:
        # If user presses Ctrl+C during the timer, on_exit will run
        pass

    # f) Timer done → clean up & dump
    disconnect_sensors()
    for st in states:
        print("dumping")
        st.close_files()
        print("dumped")
        print(f"  • Wrote {st.acc_count} accel rows → {st.acc_file}")
        print(f"  • Wrote {st.gyro_count} gyro rows → {st.gyro_file}")

    print("All done. Exiting.")
    sys.exit(0)