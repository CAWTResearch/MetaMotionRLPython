from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, sys, threading, datetime, os
from collections import deque
import torch
import torch.nn as nn
from mbientlab.warble import *
from threading import Thread, Event

import joblib, csv
import numpy as np

# Sensor y dongle MACs
# device_macs = ["F8:DC:C7:F1:48:7A", "CE:5A:39:E6:8F:B3", "F7:68:55:8D:84:0E", "FC:97:E9:E0:E8:E4", "F4:73:A1:AB:BB:64" ,"E6:AC:5E:B8:4C:D9", "E6:4F:B9:D7:18:7C", "F0:3D:E7:ED:F6:F7"] #NEW
device_macs = ["CE:5A:39:E6:8F:B3", "F7:68:55:8D:84:0E", "F8:DC:C7:F1:48:7A", "E6:4F:B9:D7:18:7C", "F0:3D:E7:ED:F6:F7", "F4:73:A1:AB:BB:64"]

dongle_macs = ['00:E0:5C:48:02:38', '00:E0:5C:48:0B:98', '00:E0:5C:48:01:21', '00:E0:5C:48:03:93', 'D8:3A:DD:EA:0C:EF'
]
# '3C:0A:F3:10:17:F0'
#'D8:3A:DD:EA:0C:EF'
states = []

buffer = deque(maxlen=50)
combinecounter = 0
predicted_event = Event()
input_dim=30 
cnn_out_channels=256
lstm_hidden=256
lstm_layers=2 
output_dim=6

QuaternionSensors = []
NormalSensors = []



profiles = [
    {"interval":8.75, "latency":0, "timeout":10000},
    {"interval":10.0, "latency":0, "timeout":10000},
    {"interval":11.25, "latency":0, "timeout":10000},
    {"interval":12.5, "latency":0, "timeout":10000},
    {"interval":13.75, "latency":0, "timeout":10000},
    {"interval":7.5, "latency":0, "timeout":10000},
]

os.makedirs('DriveUpload', exist_ok=True)
pred_file = open(os.path.join('DriveUpload', 'predictions.csv'),   'w', newline='')
pred_writer = csv.writer(pred_file)
pred_writer.writerow([
    'timestamp',
    'prediction',
    *[f'prob_{i}' for i in range(output_dim)]
])

def preprocess_data(buffer, scaler):
    data_np = np.array(buffer)  # shape (N, 30)

    flat = data_np.reshape(-1, 30)            # (N, 30)
    scaled = scaler.transform(flat)      # (N, 30)

    tensor = torch.tensor(scaled, dtype=torch.float32).unsqueeze(0)  # (1, N, 30)
    return tensor


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
        subprocess.run(["sudo","rfkill", "unblock", "bluetooth"])
        time.sleep(2)
    except Exception:
        pass

# Clase de estado para escribir CSV directamente en callbacks

class State:

    def __init__(self, device):
        self.device = device

        self.acc_count  = 0
        self.gyro_count = 0
        self.quat_count = 0

        self.acc_Y = 0 
        self.acc_X = 0
        self.acc_Z = 0 

        self.gyro_Y = 0
        self.gyro_X = 0
        self.gyro_Z = 0

        self.quat_W = 0
        self.quat_X = 0
        self.quat_Y = 0
        self.quat_Z = 0
        
        self.time = datetime.datetime.now().strftime('%H:%M:%S.%f')
        
        # Prepare callback wrappers
        self.acc_deque = deque(maxlen=1)
        self.acc_cb   = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_deque= deque(maxlen=1)
        self.gyro_cb = FnVoid_VoidP_DataP(self.gyro_data_handler)
        self.quat_deque = deque(maxlen=1)
        self.quaternion_cb = FnVoid_VoidP_DataP(self.quaternion_handler)
    
    def quaternion_handler(self, ctx, data_ptr):

        host_time = datetime.datetime.now().timestamp()
        val = parse_value(data_ptr)
        self.quat_deque.append(( val.w, val.x, val.y, val.z))
        self.quat_W, self.quat_X, self.quat_Y, self.quat_Z = val.w, val.x, val.y, val.z
        self.quat_W, self.quat_X, self.quat_Y, self.quat_Z = val.w, val.x, val.y, val.z
        # self._quat_writer.writerow([host_time, val.w, val.x, val.y, val.z])
        # self._quat_fh.flush()
        self.quat_count += 1

    def acc_data_handler(self, ctx, data_ptr):

        host_time = datetime.datetime.now().timestamp()
        # 3) parse x,y,z
        val = parse_value(data_ptr)
        self.acc_deque.append((val.x, val.y, val.z))
        self.acc_X, self.acc_Y, self.acc_Z = val.x, val.y, val.z

        # self._acc_writer.writerow([host_time, val.x, val.y, val.z])
        # self._acc_fh.flush()
        self.acc_count += 1


    def gyro_data_handler(self, ctx, data_ptr):

        host_time = datetime.datetime.now().timestamp()
        val = parse_value(data_ptr)
        self.gyro_deque.append((val.x, val.y, val.z))
        self.gyro_X, self.gyro_Y, self.gyro_Z = val.x, val.y, val.z

        # self._gyro_writer.writerow([host_time, val.x, val.y, val.z])
        # self._gyro_fh.flush()
        self.gyro_count += 1


    def get_acc_cb(self):
        return self.acc_cb

    def get_gyro_cb(self):
        return self.gyro_cb
    
    def get_quaternion_cb(self):
        return self.quaternion_cb
    
    def get_acc_Y(self):
        return self.acc_Y
    
    def get_acc_X(self):
        return self.acc_X
    
    def get_acc_Z(self):
        return self.acc_Z
    
    def get_gyro_Y(self):
        return self.gyro_Y
    
    def get_gyro_X(self):
        return self.gyro_X
    
    def get_gyro_Z(self):
        return self.gyro_Z
    
    def get_quat_W(self):
        return self.quat_W
    
    def get_quat_X(self):
        return self.quat_X
    
    def get_quat_Y(self):
        return self.quat_Y
    
    def get_quat_Z(self):
        return self.quat_Z
    
    def close_files(self):
        self._acc_fh.close()
        self._gyro_fh.close()
        self._quat_fh.close()

def assign_sensors_to_dongles(devices, dongles):
    assign = {d:[] for d in dongles}
    for i, mac in enumerate(devices):
        assign[dongles[i % len(dongles)]].append(mac)
    return assign

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
def configureQuaternions(states, Q_Quantaty):
    Sensor_Names = ["q_chest", "q_left_hand", "q_right_knee"]

    i = 0
    for st, settings in zip(states[len(NormalSensors):len(NormalSensors)+Q_Quantaty], profiles[len(NormalSensors):len(NormalSensors)+Q_Quantaty]):
        d = st.device
        print("Configuring device Quaternion" + d.address + " Type   :   " + Sensor_Names[i])

        base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
        os.makedirs(base_dir, exist_ok=True)

        # Remember each sensor's MAC (without colons) to name files
        mac_no_colon = st.device.address

        # Full paths where we’ll dump at the end:
        st.quat_file  = os.path.join(base_dir, f"quat_{mac_no_colon}.csv")

        # QUAT file
        st._quat_fh = open(st.quat_file,  "w", newline='')
        st._quat_writer = csv.writer(st._quat_fh)
        st._quat_writer.writerow(['host_time',f'{Sensor_Names[i]}_w' ,f'{Sensor_Names[i]}_x',f'{Sensor_Names[i]}_y',f'{Sensor_Names[i]}_z'])
        
        libmetawear.mbl_mw_settings_set_connection_parameters(
            d.board,
            settings["interval"],   # min & max the same
            settings["interval"],
            settings["latency"],
            settings["timeout"]
        )

        time.sleep(1.5)
        libmetawear.mbl_mw_settings_set_tx_power(d.board, 8)
        time.sleep(1.5)

        # Configuración de Sensor Fusion
        libmetawear.mbl_mw_sensor_fusion_set_mode(d.board, SensorFusionMode.IMU_PLUS)
        libmetawear.mbl_mw_sensor_fusion_set_acc_range(d.board, SensorFusionAccRange._16G)
        libmetawear.mbl_mw_sensor_fusion_set_gyro_range(d.board, SensorFusionGyroRange._2000DPS)
        libmetawear.mbl_mw_sensor_fusion_write_config(d.board)
        QuaternionSensors.append((Sensor_Names[i], st))
        i+=1
        time.sleep(0.5)
    
    return 

def configureNormal(states, N_Quantaty):

    Sensor_Names = ["n_chest", "n_left_knee", "n_right_hand"]

    i = 0
    for st in states[0:N_Quantaty]:
        b = st.device.board
        print("Configuring device Normal " + st.device.address + " Type   :   " + Sensor_Names[i])
        # Base folder for final CSVs; ensure it exists
        base_dir = os.path.join(os.path.dirname(__file__), "DriveUpload")
        os.makedirs(base_dir, exist_ok=True)

        # Remember each sensor's MAC (without colons) to name files
        mac_no_colon = st.device.address

        # Full paths where we’ll dump at the end:
        st.acc_file  = os.path.join(base_dir, f"acc_{mac_no_colon}.csv")
        st.gyro_file = os.path.join(base_dir, f"gyro_{mac_no_colon}.csv")


        # ACC file
        st._acc_fh = open(st.acc_file,  "w", newline='')
        st._acc_writer = csv.writer(st._acc_fh)
        st._acc_writer.writerow(['host_time',Sensor_Names[i]+'_acc_x', Sensor_Names[i]+'_acc_y', Sensor_Names[i]+ '_acc_z'])

        # GYRO file
        st._gyro_fh = open(st.gyro_file, "w", newline='')
        st._gyro_writer = csv.writer(st._gyro_fh)
        st._gyro_writer.writerow(['host_time', Sensor_Names[i] + '_gyro_x', Sensor_Names[i] + '_gyro_y', Sensor_Names[i] + '_gyro_z'])

        libmetawear.mbl_mw_settings_set_connection_parameters(
            b,
            profiles[i]["interval"],   # min & max the same
            profiles[i]["interval"],
            profiles[i]["latency"],
            profiles[i]["timeout"]
        )
        time.sleep(1.5)
        libmetawear.mbl_mw_settings_set_tx_power(b, 8)
        time.sleep(1.5)

        # ACC: set ODR and range
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._100Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._16G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
 

        # GYRO: set ODR and range
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._100Hz)
        libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._2000dps)
        libmetawear.mbl_mw_gyro_bmi270_write_config(b)

        NormalSensors.append((Sensor_Names[i],st))
        i+=1
    return

def subscribe_sensors():
    for Sensor in NormalSensors:
        st = Sensor[1]
        b = Sensor[1].device.board
        
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

    
    for Sensor in QuaternionSensors:
        st = Sensor[1]
        d = Sensor[1].device

        signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_datasignal_subscribe(signal_quat, None, st.get_quaternion_cb())
        libmetawear.mbl_mw_sensor_fusion_enable_data(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_sensor_fusion_start(d.board)

# Configura y suscribe sensores por separado
def configure_and_subscribe_sensors(states, Int_Quaternions, Int_Normals):
    if Int_Quaternions + Int_Normals != len(states):
        raise ValueError("The sum of Int_Quaternions and Int_Normals must equal the number of states.")
    print(str(len(states)) + " number of states")
    configureNormal(states, Int_Normals)
    configureQuaternions(states, Int_Quaternions)

    subscribe_sensors()
    return    

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

        # libmetawear.mbl_mw_debug_reset(st.device.board)
        time.sleep(2.0)
        # print("debugged")

        # 5) Finally, disconnect over BLE
        libmetawear.mbl_mw_debug_disconnect(b)

        # Give the board a moment to process each step
        time.sleep(1.0)

def reconfigure_and_subscribe(st, retries=5, backoff=1.0):
    # BlueZ will call this on disconnect; immediately spin off a thread
    threading.Thread(target=_do_reconnect, args=(st, retries, backoff), daemon=True).start()

def _do_reconnect(st, retries, backoff):
    dev     = st.device
    b       = dev.board
    mac     = dev.address
    profile = st.profile


    # 1) One big board‐side reset clears out all streams & subscriptions
    if dev.is_connected:
        return
    try:
        dev.disconnect()
    except Exception:
        pass

    timeout = time.time() + 5.0
    while dev.is_connected and time.time() < timeout:
        time.sleep(0.05)

    # 3) Give BlueZ another moment
    time.sleep(backoff)

    # 4) Retry connect() up to `retries` times
    for i in range(1, retries+1):
        try:
            dev.connect()
            if dev.is_connected:
                break
        except Exception as e:
            print(f"  • connect #{i} failed: {e}")
        time.sleep(backoff)
    else:
        return

class CNN_LSTM_Sensor(nn.Module):
    def __init__(self, input_dim, cnn_out_channels, lstm_hidden, lstm_layers, output_dim):
        super(CNN_LSTM_Sensor, self).__init__()

        self.cnn = nn.Sequential(
            nn.Conv1d(input_dim, cnn_out_channels, kernel_size=3, padding=1),
            nn.BatchNorm1d(cnn_out_channels),
            nn.ReLU(),

            nn.Conv1d(cnn_out_channels, cnn_out_channels, kernel_size=3, padding=1),
            nn.BatchNorm1d(cnn_out_channels),
            nn.ReLU(),

            nn.Conv1d(cnn_out_channels, cnn_out_channels, kernel_size=3, padding=1),
            nn.BatchNorm1d(cnn_out_channels),
            nn.ReLU(),

            nn.MaxPool1d(kernel_size=2)
        )

        self.lstm = nn.LSTM(input_size=cnn_out_channels,
                            hidden_size=lstm_hidden,
                            num_layers=lstm_layers,
                            batch_first=True)

        self.fc = nn.Sequential(
            nn.Linear(lstm_hidden, 128),
            nn.ReLU(),
            nn.Dropout(0.5),
            nn.Linear(128, 64),
            nn.ReLU(),
            nn.Linear(64, output_dim)
        )

    def forward(self, x):
        x = x.permute(0, 2, 1)   # (batch, time, features) → (batch, features, time)
        x = self.cnn(x)
        x = x.permute(0, 2, 1)   # CNN expects (batch, channels, time)
                                 # (batch, time, channels) for LSTM
        lstm_out, _ = self.lstm(x)
        x = lstm_out[:, -1, :]   # Last time step
        return self.fc(x)

def CombineData():
    data = []   
    for name, st in QuaternionSensors:
        # --- QUAT ---
        if len(st.quat_deque) >0:
                w1, x1, y1, z1 = st.quat_deque.popleft()
        else:
            w1, x1, y1, z1 = st.quat_W, st.quat_X, st.quat_Y, st.quat_Z

        data += [w1, x1, y1, z1]

    for name, st in NormalSensors:
        # --- ACC ---
        if len(st.acc_deque) >0:
            ax1, ay1, az1 = st.acc_deque.popleft()
             
        else:
            ax1, ay1, az1 = st.acc_X, st.acc_Y, st.acc_Z
            
        # --- GYRO ---    
        if len(st.gyro_deque) > 0:
            gx1, gy1, gz1 = st.gyro_deque.popleft()
        else:
            gx1, gy1, gz1 = st.gyro_X, st.gyro_Y, st.gyro_Z
        # append both accel + both gyro
        data += [
            ax1, ay1, az1,
            gx1, gy1, gz1
        ]
    buffer.append(data)  
    # return data
    return

def get_prediction(model):
    # prev_time = time.time()
    while True:
        if (len(buffer)>=50 and combinecounter>=25):
            predicted_event.set()
            print(combinecounter)
            # start_time = time.time()
            data_tensor = preprocess_data(buffer, scaler)

            with torch.no_grad():
                output = model(data_tensor)
                probabilities = torch.softmax(output, dim=1).squeeze().tolist()
                prediction = int(torch.argmax(output, dim=1).item())

            # end_time = time.time()
            # DeltaT = end_time - prev_time
            # latency = (end_time - start_time)

            print(f"Predicción: {prediction}, Probabilidades: {probabilities}")

            # print(f"[inference] Latency: {latency:.4f}s")

            # print(f"[inference] DeltaT: {DeltaT:.4f}s")
            timestamp = datetime.datetime.now().isoformat()
            pred_writer.writerow([timestamp, prediction, *probabilities])
            pred_file.flush()

            # prev_time = end_time 
            # time.sleep(0.1)

# Main loop
if __name__ == '__main__':

    force_disconnect_sensors()
    connect_sensors(device_macs, dongle_macs)
    configure_and_subscribe_sensors(states, 3, 3)
    print("All sensors configured & subscribed")

    model = CNN_LSTM_Sensor(input_dim=input_dim, cnn_out_channels=cnn_out_channels, lstm_hidden=lstm_hidden, lstm_layers=lstm_layers, output_dim=output_dim)
    print("modeled")

    # Scaler
    scaler = joblib.load("scaler_model_full_model.pkl")
    print("Scaled")

    model.load_state_dict(torch.load("cnn_lstm_fold2.pth", map_location=torch.device('cpu')))
    model.eval()
    print("Modeled again")

    data_file = open(os.path.join('DriveUpload', 'combined_data.csv'), 'w', newline='')
    data_writer = csv.writer(data_file)
    # Build combined‐data headers from your sensor lists:
    data_headers = ['host_time']
    for name,_ in QuaternionSensors:
        data_headers += [f'{name}_{axis}' for axis in ('w','x','y','z')]
    for name,_ in NormalSensors:
        data_headers += [f'{name}_acc_{ax}'  for ax in ('x','y','z')]
        data_headers += [f'{name}_gyro_{ax}' for ax in ('x','y','z')]
    data_writer.writerow(data_headers)

    try:
        print("tried")
        target_dt = 1.0 / 50

        t1 = Thread(target=get_prediction, args=(model,), daemon=True)
        t1.start()

        start_ts = time.perf_counter()
        screen_limit = 25
        count =0
        while True:
            count +=1
            elapsedtime= time.perf_counter()-start_ts
            loop_start = time.perf_counter()
            next_call = start_ts + count * target_dt
            sleep_for = next_call - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            
            # data = CombineData()
            CombineData()
            host_time = datetime.datetime.now().timestamp()
            
            # row = [host_time] + data[:30]
            # data_writer.writerow(row)
            combinecounter+=1

            if combinecounter> screen_limit and predicted_event.is_set() and len(buffer)>=50:
                combinecounter =1
                predicted_event.clear()
                print(f"{elapsedtime}")
        
    except KeyboardInterrupt:
        # If user presses Ctrl+C during the timer, on_exit will run
        pass
    finally:
        # f) Timer done → clean up & dump
        disconnect_sensors()
        pred_file.close()
        # data_file.close()

        print("All done. Exiting.")
        sys.exit(0)