from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, signal, sys, threading, datetime
from collections import deque
import torch
import torch.nn as nn
from mbientlab.warble import *
from multiprocessing import Process
from threading import Thread, Event


import joblib
import numpy as np
states = []

buffer = deque(maxlen=50)
combinecounter = 0
predicted_event = Event()
input_dim=30 
cnn_out_channels=512 
lstm_hidden=512 
lstm_layers=2 
output_dim=6
QuaternionSensors = []
NormalSensors = []



def preprocess_data(buffer, scaler):
    data_np = np.array(buffer)  # shape (N, 30)

    flat = data_np.reshape(-1, 30)            # (N, 30)
    scaled = scaler.transform(flat)      # (N, 30)

    tensor = torch.tensor(scaled, dtype=torch.float32).unsqueeze(0)  # (1, N, 30)
    return tensor

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
    Data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    
    buffer.append(Data)
    return 

def get_prediction(model):
    prev_time = time.time()
    while True:
        if len(buffer)>=50 and combinecounter>=25:
            start_time = time.time()
            data_tensor = preprocess_data(buffer, scaler)

            with torch.no_grad():
                output = model(data_tensor)
                probabilities = torch.softmax(output, dim=1).squeeze().tolist()
                prediction = int(torch.argmax(output, dim=1).item())

            end_time = time.time()
            # print(datetime.datetime.now().strftime('%H:%M:%S.%f'))
            DeltaT = end_time - prev_time
            # latency = (end_time - start_time)

            # wait    = (end_time - prev_time) - (end_time - start_time)

            print(f"Predicción: {prediction}, Probabilidades: {probabilities}")

            # print(f"[inference] Latency: {latency:.4f}s")

            print(f"[inference] DeltaT: {DeltaT:.4f}s")

            # print(f"[time between] wait: {wait:.4f}s")
            
            prev_time = end_time 
            predicted_event.set()
            time.sleep(0.02)

# Main loop
if __name__ == '__main__':

    model = CNN_LSTM_Sensor(input_dim=input_dim, cnn_out_channels=cnn_out_channels, lstm_hidden=lstm_hidden, lstm_layers=lstm_layers, output_dim=output_dim)
    model = torch.jit.script(model)
    print("modeled")

    # Scaler
    scaler = joblib.load("scaler_model_full_model.pkl")
    print("Scaled")

    model.load_state_dict(torch.load("cnn_lstm_fold1.pth", map_location=torch.device('cpu')))
    model.eval()
    print("Modeled again")
    
    # d) Allow Ctrl+C to abort early
    def on_exit(sig, frame):
        print("\nInterrupted by user!")

        sys.exit(0)

    signal.signal(signal.SIGINT, on_exit)

    try:
        print("tried")
        target_dt = 1.0 / 50

        t1 = Thread(target=get_prediction, args=(model,), daemon=True)
        t1.start()

        next_time = time.perf_counter()
        while True:
            jitter_margin = 0.002
            screen_limit = 25
            loop_start = time.perf_counter()
            deadline = next_time
            sleep =  deadline - loop_start -jitter_margin

            if sleep > 0:
                time.sleep(sleep)
            
            while time.perf_counter() < deadline:
                pass
            
            CombineData()
            combinecounter+=1
            # print(combinecounter)
            if combinecounter> screen_limit and predicted_event.is_set():
                combinecounter -= screen_limit
                predicted_event.clear()
            next_time+=(1/50)
        
    except KeyboardInterrupt:
        # If user presses Ctrl+C during the timer, on_exit will run
        pass

    # f) Timer done → clean up & dump

    print("All done. Exiting.")
    sys.exit(0)