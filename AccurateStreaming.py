from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, datetime, os, csv, signal, sys

# Sensor y dongle MACs
device_macs = ["D5:42:DD:AC:BE:E1","C4:65:87:1A:13:0B", "F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "E6:AC:5E:B8:4C:D9"]
# dongle_macs = ["3C:0A:F3:10:17:F0"]
dongle_macs = ['00:E0:5C:48:00:2F','00:E0:5C:48:06:BD','00:E0:5C:48:03:93','00:E0:5C:48:01:63','00:E0:5C:48:00:DA']
# 'D8:3A:DD:EA:0C:EF' ,
states = []
# Asegura desconexión previa
STREAM_DURATION = 60

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

        # Base folder for final CSVs; ensure it exists
        base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
        os.makedirs(base_dir, exist_ok=True)

        # Remember each sensor's MAC (without colons) to name files
        mac_no_colon = device.address

        # Full paths where we’ll dump at the end:
        self.acc_file  = os.path.join(base_dir, f"acc_{mac_no_colon}.csv")
        self.gyro_file = os.path.join(base_dir, f"gyro_{mac_no_colon}.csv")

        # Instead of creating the files now, we just create empty lists
        # that will hold tuples like (host_time, sensor_time, x, y, z).
        self.acc_data_list  = []
        self.gyro_data_list = []

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

        # 4) append to in-memory list
        self.acc_data_list.append((host_time, sensor_time, x, y, z))

    def gyro_data_handler(self, ctx, data_ptr):
        # Same as above, but for gyroscope
        sensor_time = datetime.datetime.fromtimestamp(
            data_ptr.contents.epoch / 1000.0
        ).strftime('%H:%M:%S.%f')
        host_time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z

        self.gyro_data_list.append((host_time, sensor_time, x, y, z))

    def get_acc_cb(self):
        return self.acc_cb

    def get_gyro_cb(self):
        return self.gyro_cb

    def dump_to_csv(self):
        """
        When streaming is done, call this to write both lists out to CSV.
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

        # ACC: set ODR and range
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._25Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
 

        # GYRO: set ODR and range
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._25Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._1000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)
        time.sleep(2)  # Give time for config to apply
    
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

        print("Debug reset")
        libmetawear.mbl_mw_debug_reset(st.device.board)
        time.sleep(2.0)
        print("debugged")

        # 5) Finally, disconnect over BLE
        libmetawear.mbl_mw_debug_disconnect(b)

        # Give the board a moment to process each step
        time.sleep(1.0)

    print("All disconnected")




def on_disconnect(ctx, board):
    print(f"[WARN] Lost connection to {board.address}. Attempting reconnection…")
    # tear down your state for this device, then:
    board.connect(board.address, board.hci_mac)
    
# Main loop
if __name__ == '__main__':
    force_disconnect_sensors()
    connect_sensors(device_macs, dongle_macs)
    configure_and_subscribe_sensors(states)

    # d) Allow Ctrl+C to abort early
    def on_exit(sig, frame):
        print("\nInterrupted by user!")
        disconnect_sensors()
        for st in states:
            st.dump_to_csv()
            print(f"  • Wrote {len(st.acc_data_list)} accel rows → {st.acc_file}")
            print(f"  • Wrote {len(st.gyro_data_list)} gyro rows → {st.gyro_file}")
        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    # e) Timer loop
    print(f"Streaming 50 Hz from each sensor for {STREAM_DURATION} seconds…")
    start_ts = time.time()
    try:
        while True:
            if time.time() - start_ts >= STREAM_DURATION:
                print(f"\n{STREAM_DURATION} seconds elapsed. Stopping streaming…")
                break
            time.sleep(0.1)
    except KeyboardInterrupt:
        # If user presses Ctrl+C during the timer, on_exit will run
        pass

    # f) Timer done → clean up & dump
    disconnect_sensors()
    for st in states:
        print("dumping")
        st.dump_to_csv()
        print("dumped")
        print(f"  • Wrote {len(st.acc_data_list)} accel rows → {st.acc_file}")
        print(f"  • Wrote {len(st.gyro_data_list)} gyro rows → {st.gyro_file}")

    print("All done. Exiting.")
    sys.exit(0)