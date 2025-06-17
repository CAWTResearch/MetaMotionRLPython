from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, datetime, os, csv, signal, sys, threading, glob
from collections import deque
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer
import torch
import torch.nn as nn
from mbientlab.warble import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import joblib
import numpy as np

# Sensor y dongle MACs
# device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "E6:AC:5E:B8:4C:D9",'F8:DC:C7:F1:48:7A',"E6:4F:B9:D7:18:7C"]
# device_macs = ['F8:DC:C7:F1:48:7A',"E6:4F:B9:D7:18:7C"]
device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "D5:42:DD:AC:BE:E1", "E6:4F:B9:D7:18:7C" ,"E6:AC:5E:B8:4C:D9"]
# device_macs = ["F0:3D:E7:ED:F6:F7", "CE:5A:39:E6:8F:B3", "E6:AC:5E:B8:4C:D9"]
dongle_macs = ['00:E0:5C:48:00:DA','00:E0:5C:48:01:63', 'D8:3A:DD:EA:0C:EF', '00:E0:5C:48:03:93', '00:E0:5C:48:06:BD']
# dongle_macs = ['D8:3A:DD:EA:0C:EF', '00:E0:5C:48:03:93', '00:E0:5C:48:06:BD']
# dongle_macs = ["00:E0:5C:48:01:70","00:E0:5C:48:02:38", "00:E0:5C:48:01:34", "00:E0:5C:48:0B:98", "00:E0:5C:48:00:DA"]

states = []

input_dim=30 
cnn_out_channels=512 
lstm_hidden=512 
lstm_layers=2 
output_dim=6
QuaternionSensors = []
NormalSensors = []

profiles = [
    {"interval":25.0, "latency":5, "timeout":10000},
    {"interval":30.0, "latency":4, "timeout":10000},
    {"interval":35.0, "latency":3, "timeout":10000},
    {"interval":40.0, "latency":2, "timeout":10000},
    {"interval":7.5, "latency":1, "timeout":10000},
]


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

        # Remember each sensor's MAC (without colons) to name files
        mac_no_colon = device.address

        
        # Prepare callback wrappers
        self.acc_cb  = FnVoid_VoidP_DataP(self.acc_data_handler)
        self.gyro_cb = FnVoid_VoidP_DataP(self.gyro_data_handler)
        self.quaternion_cb = FnVoid_VoidP_DataP(self.quaternion_handler)
        


    def acc_data_handler(self, ctx, data_ptr):
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z

        self.acc_X = x
        self.acc_Y = y
        self.acc_Z = z

        self.acc_count += 1


    def gyro_data_handler(self, ctx, data_ptr):
        val = parse_value(data_ptr)
        x, y, z = val.x, val.y, val.z

        self.gyro_X = x
        self.gyro_Y = y
        self.gyro_Z = z

       
        self.gyro_count += 1

    def quaternion_handler(self, ctx, data_ptr):
        val = parse_value(data_ptr)
        w, x, y, z = val.w, val.x, val.y, val.z

        self.quat_W = w
        self.quat_X = x 
        self.quat_Y = y
        self.quat_Z = z

        self.quat_count += 1



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
def configureQuaternions(states, Q_Quantaty):
    Sensor_Names = ["q_chest", "q_left_hand", "q_right_knee"]

    i = 0
    for st, settings in zip(states[len(NormalSensors)::len(NormalSensors)+Q_Quantaty], profiles[len(NormalSensors)::len(NormalSensors)+Q_Quantaty]):
        d = st.device
        print("Configuring device Quaternion" + d.address)

        
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
    
    return 

def configureNormal(states, N_Quantaty):

    Sensor_Names = ["n_chest", "n_left_hand", "n_right_knee"]


    i = 0
    for st, settings in zip(states[::N_Quantaty], profiles[::N_Quantaty]):
        b = st.device.board
        print("Configuring device Quaternion" + st.device.address + " Type   :   " + Sensor_Names[i])


        libmetawear.mbl_mw_settings_set_connection_parameters(
            b,
            settings["interval"],   # min & max the same
            settings["interval"],
            settings["latency"],
            settings["timeout"]
        )
        time.sleep(1.5)
        libmetawear.mbl_mw_settings_set_tx_power(b, 8)
        time.sleep(1.5)

        # ACC: set ODR and range
        libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._50Hz)
        libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._16G)
        libmetawear.mbl_mw_acc_write_acceleration_config(b)
 

        # GYRO: set ODR and range
        libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._50Hz)
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
        libmetawear.mbl_mw_datasignal_subscribe(signal_quat, None, st.quaternion_cb())
        libmetawear.mbl_mw_sensor_fusion_enable_data(d.board, SensorFusionData.QUATERNION)
        libmetawear.mbl_mw_sensor_fusion_start(d.board)

# Configura y suscribe sensores por separado
def configure_and_subscribe_sensors(states, Int_Quaternions, Int_Normals):
    if Int_Quaternions + Int_Normals != len(states):
        raise ValueError("The sum of Int_Quaternions and Int_Normals must equal the number of states.")
    
    configureNormal(states, Int_Normals)
    configureQuaternions(states, Int_Quaternions)

    subscribe_sensors()
    return    

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


# Main loop
if __name__ == '__main__':


    force_disconnect_sensors()
    connect_sensors(device_macs, dongle_macs)
    configure_and_subscribe_sensors(states, 3, 3)

    model = CNN_LSTM_Sensor(input_dim=input_dim, cnn_out_channels=cnn_out_channels, lstm_hidden=lstm_hidden, lstm_layers=lstm_layers, output_dim=output_dim)
    
    # Scaler
    scaler = joblib.load("minmax_scaler.pkl")
    print("✅ Scaler cargado")

    model.load_state_dict(torch.load("best_model_89.pth", map_location=torch.device('cpu')))
    model.eval()
    print("✅ Modelo cargado")

    
    # d) Allow Ctrl+C to abort early
    def on_exit(sig, frame):
        print("\nInterrupted by user!")
        disconnect_sensors()

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

    print("All done. Exiting.")
    sys.exit(0)