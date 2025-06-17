import sys
from time import sleep
from threading import Event
from collections import deque
from PyQt5.QtWidgets import QApplication, QWidget, QLabel, QVBoxLayout
from PyQt5.QtCore import QTimer
import torch
import torch.nn as nn
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.warble import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

import joblib
import numpy as np

# ========== Modelo CNN-LSTM ==========
class CNN_LSTM_Model(nn.Module):
    def __init__(self, input_size, cnn_channels=16, hidden_size=32, num_classes=4):
        super().__init__()
        self.conv1 = nn.Conv1d(in_channels=input_size, out_channels=cnn_channels, kernel_size=10, padding=5)
        self.bn1 = nn.BatchNorm1d(cnn_channels)
        self.relu = nn.ReLU()
        self.dropout = nn.Dropout(0.3)

        self.lstm = nn.LSTM(input_size=cnn_channels, 
                            hidden_size=hidden_size, 
                            batch_first=True, 
                            bidirectional=False)
        
        self.fc = nn.Linear(hidden_size, num_classes)

    def forward(self, x):
        x = x.permute(0, 2, 1)   
        x = self.relu(self.bn1(self.conv1(x)))       
        x = self.dropout(x)
        x = x.permute(0, 2, 1)
        out, _ = self.lstm(x)
        out = self.fc(out[:, -1, :])
        return out

# ========== Manejo del Sensor ==========
class SensorState:
    def __init__(self, device):
        self.device = device
        self.acc_data = None
        self.gyro_data = None
        self.buffer = deque(maxlen=300)
        self.acc_callback = FnVoid_VoidP_DataP(self.data_handler_acc)
        self.gyro_callback = FnVoid_VoidP_DataP(self.data_handler_gyro)

    def data_handler_acc(self, ctx, data):
        self.acc_data = parse_value(data)
        self._combine_data()

    def data_handler_gyro(self, ctx, data):
        self.gyro_data = parse_value(data)
        self._combine_data()

    def _combine_data(self):
        if self.acc_data and self.gyro_data:
            combined = [
                self.acc_data.x, self.acc_data.y, self.acc_data.z,
                self.gyro_data.x, self.gyro_data.y, self.gyro_data.z
            ]
            self.buffer.append(combined)
            self.acc_data = None
            self.gyro_data = None

    def start_stream(self):
        print("🔄 Configuring sensor stream...")
        libmetawear.mbl_mw_settings_set_connection_parameters(self.device.board, 7.5, 7.5, 0, 6000)
        sleep(1.0)

        # Accelerometer config
        libmetawear.mbl_mw_acc_set_odr(self.device.board, 50.0)
        libmetawear.mbl_mw_acc_bosch_set_range(self.device.board, AccBoschRange._4G)
        libmetawear.mbl_mw_acc_write_acceleration_config(self.device.board)

        # Gyroscope config
        libmetawear.mbl_mw_gyro_bmi160_set_range(self.device.board, GyroBoschRange._2000dps)
        libmetawear.mbl_mw_gyro_bmi160_set_odr(self.device.board, GyroBoschOdr._50Hz)
        libmetawear.mbl_mw_gyro_bmi160_write_config(self.device.board)

        # Data signal subscription
        acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        gyro_signal = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_subscribe(acc_signal, None, self.acc_callback)
        libmetawear.mbl_mw_datasignal_subscribe(gyro_signal, None, self.gyro_callback)

        # Start streaming
        libmetawear.mbl_mw_acc_enable_acceleration_sampling(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_enable_rotation_sampling(self.device.board)
        libmetawear.mbl_mw_acc_start(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_start(self.device.board)
        print("✅ Sensor streaming started.")

    def stop_stream(self):
        print("⛔ Stopping sensor stream...")
        libmetawear.mbl_mw_acc_stop(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_stop(self.device.board)
        libmetawear.mbl_mw_acc_disable_acceleration_sampling(self.device.board)
        libmetawear.mbl_mw_gyro_bmi160_disable_rotation_sampling(self.device.board)

        acc_signal = libmetawear.mbl_mw_acc_get_acceleration_data_signal(self.device.board)
        gyro_signal = libmetawear.mbl_mw_gyro_bmi160_get_rotation_data_signal(self.device.board)
        libmetawear.mbl_mw_datasignal_unsubscribe(acc_signal)
        libmetawear.mbl_mw_datasignal_unsubscribe(gyro_signal)
        print("✅ Sensor streaming stopped.")

    def disconnect(self):
        libmetawear.mbl_mw_debug_disconnect(self.device.board)
        self.device.disconnect()
        print("🔌 Sensor disconnected.")

class App(QWidget):
    def __init__(self, model, sensor, scaler):
        super().__init__()
        self.model = model
        self.sensor = sensor
        self.scaler = scaler
        self.class_names = ["Arresto Momentum", "Expecto Patronum", "Expelliarmus", "Ready"]
        self.initUI()

    def initUI(self):
        self.label = QLabel('Prediction: Waiting...', self)

        # === Figure 1: Accelerometer Plot ===
        self.fig1 = Figure(figsize=(5, 2))
        self.canvas1 = FigureCanvas(self.fig1)
        self.ax1 = self.fig1.add_subplot(111)
        self.ax1.set_ylim(-4, 4)
        self.ax1.set_xlim(0, 50)
        self.ax1.set_title("Accelerometer")
        self.ax1.grid(True)

        # === Figure 2: Prediction Probabilities ===
        self.fig2 = Figure(figsize=(5, 2))
        self.canvas2 = FigureCanvas(self.fig2)
        self.ax2 = self.fig2.add_subplot(111)
        self.bars = self.ax2.bar(self.class_names, [0.0]*4)
        self.ax2.set_ylim(0, 1)
        self.ax2.set_title("Prediction Probabilities")

        # === Layout ===
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.canvas1)
        layout.addWidget(self.canvas2)
        self.setLayout(layout)

        self.setWindowTitle('Real-Time Classification')
        self.setGeometry(100, 100, 700, 600)

        # Timer cada 100 ms
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # cada 100 ms
        self.show()

    def preprocess_data(self, buffer):
        data_np = np.array(buffer)  # shape (50, 6)

        # Normalizar usando el scaler cargado
        flat = data_np.reshape(-1, 6)            # (50, 6)
        scaled = self.scaler.transform(flat)     # (50, 6)

        # Convertir a tensor
        tensor = torch.tensor(scaled, dtype=torch.float32).unsqueeze(0)  # (1, 50, 6)
        return tensor
    
    def get_sliding_windows(self, data, window_size=30, step_size=5):
        windows = []
        for start in range(0, len(data) - window_size + 1, step_size):
            end = start + window_size
            window = data[start:end]
            windows.append(window)
        return windows

    def update_ui(self):
        if len(self.sensor.buffer) >= 30:
            buffer_data = list(self.sensor.buffer)
            windows = self.get_sliding_windows(buffer_data, window_size=30, step_size=5)

            for window in windows:
                # === Preprocesamiento e inferencia ===
                data_tensor = self.preprocess_data(window)
                with torch.no_grad():
                    output = self.model(data_tensor)
                    probabilities = torch.softmax(output, dim=1).squeeze().tolist()
                    prediction = int(torch.argmax(output, dim=1).item())

                self.label.setText(f'Prediction: {self.class_names[prediction]}')
                # print(f'🧠 Pred: {self.class_names[prediction]} | Prob: {probabilities}')
                print(f'🧠 Pred: {self.class_names[prediction]}')

                # Mostrar solo una ventana para graficar (última)
                if window == windows[-1]:
                    # acc_x = [d[0] for d in window]
                    # acc_y = [d[1] for d in window]
                    # acc_z = [d[2] for d in window]
                    normalized_window = self.preprocess_data(window).squeeze(0).numpy()
                    acc_x = normalized_window[:, 0]
                    acc_y = normalized_window[:, 1]
                    acc_z = normalized_window[:, 2]


                    self.ax1.clear()
                    self.ax1.plot(acc_x, label='X')
                    self.ax1.plot(acc_y, label='Y')
                    self.ax1.plot(acc_z, label='Z')
                    self.ax1.set_ylim(-1, 1)
                    self.ax1.set_xlim(0, 30)
                    self.ax1.set_title("Accelerometer")
                    self.ax1.legend()
                    self.ax1.grid(True)
                    self.canvas1.draw()

                for i, bar in enumerate(self.bars):
                    bar.set_height(probabilities[i])
                self.ax2.set_ylim(0, 1)
                self.ax2.set_title("Prediction Probabilities")
                self.canvas2.draw()

# ========== Main ==========
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 script.py [MAC_ADDRESS]")
        sys.exit(1)

    mac_address = sys.argv[1]

    # Modelo
    input_size = 6
    model = CNN_LSTM_Model(input_size=input_size, cnn_channels=16, hidden_size=32, num_classes=4)
    # Scaler
    scaler = joblib.load("minmax_scaler.pkl")
    print("✅ Scaler cargado")

    model.load_state_dict(torch.load("best_model_89.pth", map_location=torch.device('cpu')))
    model.eval()
    print("✅ Modelo cargado")

    # Sensor
    sensor_device = MetaWear(mac_address)
    sensor_device.connect()
    sensor = SensorState(sensor_device)
    sensor.start_stream()

    # App
    app = QApplication(sys.argv)
    ex = App(model, sensor, scaler)
    app.exec_()

    # Finalizar
    sensor.stop_stream()
    sleep(1)
    sensor.disconnect()
