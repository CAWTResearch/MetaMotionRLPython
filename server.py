from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value
from mbientlab.metawear.cbindings import *
from mbientlab.metawear.cbindings import (
    FnVoid_VoidP_DataP,
    AccBmi270Odr, AccBoschRange,
    GyroBoschOdr, GyroBoschRange
)
import subprocess, time, sys, threading, datetime
from collections import deque
import torch
import torch.nn as nn
from mbientlab.warble import *
from threading import Thread, Event, Lock
import asyncio, websockets
import joblib
import re
import numpy as np
import json
from typing import Dict, List, Tuple, Set, Any, Set, Optional
from websockets.server import WebSocketServerProtocol
import csv, io, time, json, zlib, hashlib, asyncio

CONNECTED: Set[WebSocketServerProtocol] = set()
WS_LOOP: Optional[asyncio.AbstractEventLoop] = None

states: List["State"] = []
NormalSensors: List[Tuple[str, "State"]] = []
QuaternionSensors: List[Tuple[str, "State"]] = []

deviceMacs: List[str] = [] 
config_lock = Lock()

streaming_event = Event()     
configured_event = Event()   

ALLOWED_MODES = {"Start Streaming", "Standby", "Stop Streaming", "None", "Calibration", "Diagnostics"}

buffer = deque(maxlen=50)
predictions_log = deque(maxlen=100000)
sample_log = deque(maxlen=2500000)
combinecounter = 0
predicted_event = Event()
input_dim=30 
cnn_out_channels=256
lstm_hidden=256
lstm_layers=2 
output_dim=6

PRED_HEADER = ["timestamp_iso","timestamp_unix","prediction","prob_0","prob_1","prob_2","prob_3","prob_4","prob_5"]
SAMP_HEADER = ["index"] + [f"x{i}" for i in range(30)]


POSITIONS = [
    'Chest-left', 'Chest-right',
    'Arm-left', 'Arm-right',
    'Knee-left', 'Knee-right'
]

ROLE_BY_POSITION = {
    'Chest-left':  'normal',
    'Chest-right': 'quat',
    'Arm-left':    'normal',
    'Arm-right':   'quat',
    'Knee-left':   'normal',
    'Knee-right':  'quat',
}

CurrentPrediction = [0]

profiles = [
    {"interval":8.75, "latency":0, "timeout":10000},
    {"interval":10.0, "latency":0, "timeout":10000},
    {"interval":11.25, "latency":0, "timeout":10000},
    {"interval":12.5, "latency":0, "timeout":10000},
    {"interval":13.75, "latency":0, "timeout":10000},
    {"interval":7.5, "latency":0, "timeout":10000},
]

CHUNK_BYTES = 64 * 1024

async def stream_csv(ws, filename: str, header: list[str], row_iter):
    stream_id = f"csv-{int(time.time())}"
    sha = hashlib.sha256()
    total = 0

    # tell client a CSV stream is starting
    await ws.send(json.dumps({
        "type": "file_start",
        "stream_id": stream_id,
        "filename": filename,
        "mime": "text/csv;charset=utf-8"
    }))

    buf = io.StringIO()
    w = csv.writer(buf)
    w.writerow(header)

    def flush_buf():
        nonlocal total
        data = buf.getvalue().encode("utf-8")
        if not data:
            return
        sha.update(data); total += len(data)
        # send as BINARY frame
        return ws.send(data)

    # write rows, chunk by size
    pending = None
    for row in row_iter:
        w.writerow(row)
        if buf.tell() >= CHUNK_BYTES:
            if pending: await pending
            pending = flush_buf()
            buf.seek(0); buf.truncate(0)

    # final flush
    if pending: await pending
    if buf.tell():
        await ws.send(buf.getvalue().encode("utf-8"))
        sha.update(buf.getvalue().encode("utf-8"))
        total += len(buf.getvalue().encode("utf-8"))

    # tell client we’re done
    await ws.send(json.dumps({
        "type": "file_end",
        "stream_id": stream_id,
        "total_bytes": total,
        "sha256": sha.hexdigest()
    }))

def iter_predictions_rows():
    # snapshot to avoid mutation during send
    snap = list(predictions_log)
    for r in snap:
        # r = {"ts": float, "prediction": int, "probabilities": [floats]}
        ts_iso = time.strftime("%Y-%m-%dT%H:%M:%S", time.gmtime(r["ts"]))  # or datetime.utcfromtimestamp(...)
        probs = (r.get("probabilities") or [])
        # pad/truncate to 6 as per your model
        probs = list(probs[:6]) + [""] * max(0, 6 - len(probs))
        yield [ts_iso, f'{r["ts"]:.6f}', r["prediction"], *probs]

def iter_samples_rows():
    for i, data in enumerate(list(sample_log)):
        yield [i] + list(data)  # simple index + 30 features


def detect_dongle_macs() -> List[str]:
    macs_by_hci: List[Tuple[int, str]] = []
    # --- Try hcitool dev ---
    try:
        res = subprocess.run(["hcitool", "dev"], capture_output=True, text=True, check=False)
        for line in res.stdout.splitlines():
            parts = line.strip().split()
            # Expected lines look like: "hci0    00:E0:5C:48:03:93"
            if len(parts) == 2 and parts[0].startswith("hci"):
                hci_name, mac = parts
                if re.fullmatch(r"[0-9A-Fa-f:]{17}", mac):
                    try:
                        idx = int(hci_name[3:])
                        macs_by_hci.append((idx, mac.upper()))
                    except ValueError:
                        pass
    except FileNotFoundError:
        pass

    # Sort by hci index and dedupe while preserving order
    macs_by_hci.sort(key=lambda t: t[0])
    seen = set()
    ordered = []
    for _, mac in macs_by_hci:
        if mac not in seen:
            seen.add(mac)
            ordered.append(mac)
    print(ordered)
    return ordered

dongle_macs = detect_dongle_macs()
if not dongle_macs:
    print("[WARN] No Bluetooth adapters detected via hcitool or bluetoothctl; connections may fail.")

def try_parse_json(raw: str):

    if not isinstance(raw, str):
        return None
    # First attempt: raw as JSON
    try:
        obj = json.loads(raw)
    except Exception:
        obj = None

    # If obj is a string (e.g., a JSON blob inside quotes), try decoding again
    if isinstance(obj, str):
        try:
            obj2 = json.loads(obj)
            obj = obj2
        except Exception:
            pass

    return obj if isinstance(obj, dict) else None


async def server(ws):
    print("Client connected", flush=True)
    CONNECTED.add(ws)
    try:
        async for raw in ws:
            print(f"{raw[:120]}...", flush=True)  # trim for sanity
            payload = try_parse_json(raw)

            if payload and payload.get("action") == "get_predictions":
                await ws.send(json.dumps({
                    "type": "predictions_dump",
                    "count": len(predictions_log),
                    "data": list(predictions_log),
                }))
                await ws.send(json.dumps({
                    "type": "sample_dump",
                    "count": len(sample_log),
                    "data":  list(sample_log)
                }))
                continue

            if payload and (payload.get("action") == "set_mode" or "mode" in payload):
                await handle_mode(ws, payload.get("mode", ""))
                continue

            if isinstance(raw, str):
                candidate = normalize_mode(raw)
                if candidate in ALLOWED_MODES:
                    print("Mode found!")
                    await handle_mode(ws, candidate)
                    continue
            await ws.send('"MAPPING_APPLIED"')

            # ---- CSV downloads (on-demand) ----
            if payload and payload.get("action") == "download_csv":
                which = payload.get("which")
                try:
                    if which == "predictions":
                        await stream_csv(ws,
                            filename=f"predictions_{int(time.time())}.csv",
                            header_cols=PRED_HEADER,
                            row_iter=iter_predictions_rows()
                        )
                    elif which == "samples":
                        await stream_csv(ws,
                            filename=f"samples_{int(time.time())}.csv",
                            header_cols=SAMP_HEADER,
                            row_iter=iter_samples_rows()
                        )
                    else:
                        await ws.send(json.dumps({"type":"error","message":"unknown CSV type"}))
                except Exception as e:
                    await ws.send(json.dumps({"type":"error","message":f"csv_stream_failed:{e}"}))
                continue

            
            if payload and (
                payload.get("action") == "set_mapping" or any(k in POSITIONS for k in payload.keys())
            ):
                mapping = payload.get("mapping") if payload.get("action") == "set_mapping" else payload
                mapping = mapping or {}

                print(f"[MAP] received keys={list(mapping.keys())}", flush=True)
                plan = plan_from_mapping(mapping)
                normals = plan["normals"]; quats = plan["quats"]; macs = plan["device_macs"]
                print(f"[MAP] normals={len(normals)} quats={len(quats)} macs={len(macs)}", flush=True)

                if len(macs) == 0:
                    await ws.send('"MAPPING_EMPTY"')
                    continue

                with config_lock:
                    # clear any previous config
                    streaming_event.clear()
                    configured_event.clear()
                    stop_subscriptions()
                    disconnect_sensors()
                    NormalSensors.clear()
                    QuaternionSensors.clear()
                    states.clear()

                    # use provided MACs (order no longer matters)
                    deviceMacs[:] = macs

                    force_disconnect_sensors()
                    connect_sensors(deviceMacs, dongle_macs)

                    expected = len(normals) + len(quats)
                    if not states or len(states) != expected:
                        await ws.send(f"CONNECT_RESULT: connected={len(states)}, expected={expected}")

                    # configure by MAC membership (correctly matches normal vs quat)
                    configure_sensors(states, quats, normals)
                    configured_event.set()
                
                    # --- mode as JSON: {"mode":"Start Streaming"} or {"action":"set_mode","mode":"Standby"} ---
                    if payload and ("mode" in payload or payload.get("action") == "set_mode"):
                        mode_value = payload.get("mode")
                        if isinstance(mode_value, str):
                            await handle_mode(ws, mode_value)
                            continue

    except websockets.ConnectionClosed:
        print("Client disconnected")
    finally:
        CONNECTED.discard(ws)

def mode(message):
    modes = {'"Standby"': 0,
             '"Start Streaming"': 1, '"Calibration"': 2, '"Diagnostics"': 3}
    if not message in modes:
        return str(-1)

    return str(modes[message])

async def handle_mode(ws, mode_value: str):
    mode = normalize_mode(mode_value)
    print(f"[MODE] selected={mode}", flush=True)

    if mode == "Start Streaming":
        if streaming_event.is_set():
            await ws.send("STREAMING_ALREADY_STARTED"); return
        if start_streaming_now():
            await ws.send("STREAMING_STARTED")
            predictions_log.clear()
        else:
            await ws.send("NOT_CONFIGURED")
        return

    if mode in {"Standby", "Stop Streaming", "None"}:
        if streaming_event.is_set():
            stop_streaming_now(); await ws.send("STREAMING_STOPPED")
            await ws.send(json.dumps({
                "type": "predictions_dump",
                "count": len(predictions_log),
                "data": list(predictions_log)  
            }))
        else:
            await ws.send("STREAMING_ALREADY_STOPPED")
        return

    if mode in {"Calibration"}:
        if streaming_event.is_set():
            stop_streaming_now()
        await ws.send(f"MODE_SET:{mode}")
        return
    await ws.send("UNKNOWN_MODE")


# def get_realtime_info():
#     # gather whatever you need here; stub:
#     print("Gathering realtime info...")
#     return "0"

async def start_ws_server():
    global WS_LOOP
    WS_LOOP = asyncio.get_running_loop()
    async with websockets.serve(server, "0.0.0.0", 8765):
        print("Server listening on 0.0.0.0:8765")
        await asyncio.Future()  # run forever
    
async def broadcast(obj):
    msg = json.dumps(obj, separators=(",", ":"))
    dead = []
    for ws in list(CONNECTED):
        try:
            await ws.send(msg)
        except Exception:
            dead.append(ws)
    for ws in dead:
        CONNECTED.discard(ws)

POSITION_BY_MAC: Dict[str, str] = {}

def normalize_mac(mac: str) -> str:
    return (mac or "").strip().upper()

def plan_from_mapping(mapping: Dict[str, str]) -> Dict[str, Any]:
    print("Received mapping!", flush=True)
    selected: List[Tuple[str, str]] = []
    seen: Set[str] = set()

    for pos in POSITIONS:
        mac = normalize_mac(mapping.get(pos, ""))
        if not mac:
            continue
        if mac in seen:
            # skip duplicates quietly
            continue
        seen.add(mac)
        selected.append((pos, mac))

    normals = [(pos, mac) for (pos, mac) in selected if ROLE_BY_POSITION.get(pos) == 'normal']
    quats   = [(pos, mac) for (pos, mac) in selected if ROLE_BY_POSITION.get(pos) == 'quat']

    # Remember MAC -> position for naming/logging/config
    POSITION_BY_MAC.clear()
    for pos, mac in selected:
        POSITION_BY_MAC[mac] = pos

    device_macs = [mac for _, mac in selected]

    return {
        "normals": normals,
        "quats":   quats,
        "device_macs": device_macs,
    }

def normalize_mode(s: str) -> str:
    if not isinstance(s, str):
        return ""
    t = s.strip()
    # This turns "\"Start Streaming\"" -> '"Start Streaming"' -> 'Start Streaming'
    for _ in range(3):
        try:
            x = json.loads(t)
        except Exception:
            break
        if isinstance(x, str):
            t = x.strip()
            continue
        break
    # Final safety: strip any surrounding quotes
    while len(t) >= 2 and t[0] == t[-1] == '"':
        t = t[1:-1].strip()

    return t


def start_streaming_now() -> bool:
    if not configured_event.is_set():
        print("[MODE] start requested but NOT_CONFIGURED", flush=True)
        return False
    with config_lock:
        print("[MODE] starting streaming...", flush=True)
        buffer.clear()
        global combinecounter
        combinecounter = 0
        subscribe_sensors()
        streaming_event.set()
    return True


def stop_streaming_now() -> bool:
    with config_lock:
        print("[MODE] stopping streaming...", flush=True)
        streaming_event.clear()
        stop_subscriptions()
        buffer.clear()                 # drop any queued frames
        global combinecounter
        combinecounter = 0             # reset frame counter
        predicted_event.clear()
    return True

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
        self.quat_count += 1

    def acc_data_handler(self, ctx, data_ptr):

        host_time = datetime.datetime.now().timestamp()
        val = parse_value(data_ptr)
        self.acc_deque.append((val.x, val.y, val.z))
        self.acc_X, self.acc_Y, self.acc_Z = val.x, val.y, val.z
        self.acc_count += 1


    def gyro_data_handler(self, ctx, data_ptr):

        host_time = datetime.datetime.now().timestamp()
        val = parse_value(data_ptr)
        self.gyro_deque.append((val.x, val.y, val.z))
        self.gyro_X, self.gyro_Y, self.gyro_Z = val.x, val.y, val.z
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

def connect_sensors(devices, dongles, retries=3):
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
                        time.sleep(1)
                        states.append(st)
                        break
                except Exception as e:
                    print(f"Conn err {mac}: {e}")
                    time.sleep(3)
    return states
def configureNormal(st: "State", name: str):
    b = st.device.board
    print(f"Configuring device Normal {st.device.address} Type: {name}")

    settings = st.profile
    libmetawear.mbl_mw_settings_set_connection_parameters(
        b, settings["interval"], settings["interval"], settings["latency"], settings["timeout"]
    )
    time.sleep(1.0)
    libmetawear.mbl_mw_settings_set_tx_power(b, 8)
    time.sleep(0.5)

    libmetawear.mbl_mw_acc_bmi270_set_odr(b, AccBmi270Odr._100Hz)
    libmetawear.mbl_mw_acc_bosch_set_range(b, AccBoschRange._16G)
    libmetawear.mbl_mw_acc_write_acceleration_config(b)

    libmetawear.mbl_mw_gyro_bmi270_set_odr(b, GyroBoschOdr._100Hz)
    libmetawear.mbl_mw_gyro_bmi270_set_range(b, GyroBoschRange._2000dps)
    libmetawear.mbl_mw_gyro_bmi270_write_config(b)

    NormalSensors.append((name, st))
    time.sleep(0.3)

def configureQuaternions(st: "State", name: str):
    d = st.device
    print(f"Configuring device Quaternion {d.address} Type: {name}")

    settings = st.profile
    libmetawear.mbl_mw_settings_set_connection_parameters(
        d.board, settings["interval"], settings["interval"], settings["latency"], settings["timeout"]
    )
    time.sleep(1.0)
    libmetawear.mbl_mw_settings_set_tx_power(d.board, 8)
    time.sleep(0.5)

    libmetawear.mbl_mw_sensor_fusion_set_mode(d.board, SensorFusionMode.IMU_PLUS)
    libmetawear.mbl_mw_sensor_fusion_set_acc_range(d.board, SensorFusionAccRange._16G)
    libmetawear.mbl_mw_sensor_fusion_set_gyro_range(d.board, SensorFusionGyroRange._2000DPS)
    libmetawear.mbl_mw_sensor_fusion_write_config(d.board)

    QuaternionSensors.append((name, st))
    time.sleep(0.3)

def configure_sensors(states_list: List["State"],
                      quats: List[Tuple[str, str]],
                      normals: List[Tuple[str, str]]):
    """
    quats/normals are [ (position, mac), ... ] from plan_from_mapping().
    We configure each connected state by checking its MAC membership.
    """
    print("Configuring Sensors!")
    quat_map   = {normalize_mac(mac): pos for (pos, mac) in quats}
    normal_map = {normalize_mac(mac): pos for (pos, mac) in normals}

    cfg_normals = 0
    cfg_quats   = 0

    for st in states_list:
        mac = normalize_mac(st.device.address)
        if mac in normal_map:
            pos = normal_map[mac]
            name = f"n_{pos.replace('-', '_')}"
            configureNormal(st, name)
            cfg_normals += 1
        elif mac in quat_map:
            pos = quat_map[mac]
            name = f"q_{pos.replace('-', '_')}"
            configureQuaternions(st, name)
            cfg_quats += 1
        else:
            print(f"Skipping {mac}: not present in mapping.")

    print(f"{len(states_list)} states; configured {cfg_normals} normals + {cfg_quats} quats")



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

def stop_subscriptions():
    # normals
    for _, st in NormalSensors:
        b = st.device.board
        try:
            libmetawear.mbl_mw_acc_stop(b)
            libmetawear.mbl_mw_acc_disable_acceleration_sampling(b)
            libmetawear.mbl_mw_gyro_bmi270_stop(b)
            libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(b)
            acc_signal  = libmetawear.mbl_mw_acc_get_acceleration_data_signal(b)
            gyro_signal = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(b)
            libmetawear.mbl_mw_datasignal_unsubscribe(acc_signal)
            libmetawear.mbl_mw_datasignal_unsubscribe(gyro_signal)
        except Exception:
            pass
    # quats
    for _, st in QuaternionSensors:
        d = st.device
        try:
            signal_quat = libmetawear.mbl_mw_sensor_fusion_get_data_signal(d.board, SensorFusionData.QUATERNION)
            libmetawear.mbl_mw_sensor_fusion_stop(d.board)
            libmetawear.mbl_mw_datasignal_unsubscribe(signal_quat)
        except Exception:
            pass

def disconnect_sensors():
    if not states:
        return
    
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
    sample_log.append(data)
    # return data
    return

def get_prediction(model):
    while True:
        if (len(buffer)>=50 and combinecounter>=25):
            predicted_event.set()
            print(combinecounter)
            data_tensor = preprocess_data(buffer, scaler)

            with torch.no_grad():
                output = model(data_tensor)
                probabilities = torch.softmax(output, dim=1).squeeze().tolist()
                prediction = int(torch.argmax(output, dim=1).item())

            print(f"Predicción: {prediction}, Probabilidades: {probabilities}")

            CurrentPrediction[0] = prediction
            c_time = time.time()
            if WS_LOOP is not None:
                payload = {
                    "type": "prediction",
                    "prediction": prediction,
                    "probabilities": probabilities,
                    "time": c_time,
                }
                asyncio.run_coroutine_threadsafe(broadcast(payload), WS_LOOP)
                predictions_log.append({
                    "ts": c_time,
                    "prediction": prediction,
                    "probabilities": probabilities
                })
        else:
            time.sleep(0.002)

if __name__ == "__main__":
    print("Starting server...")
    # Load model & scaler up front
    model = CNN_LSTM_Sensor(input_dim=input_dim, cnn_out_channels=cnn_out_channels,
                            lstm_hidden=lstm_hidden, lstm_layers=lstm_layers, output_dim=output_dim)
    scaler = joblib.load("scaler_model_full_model.pkl")
    model.load_state_dict(torch.load("cnn_lstm_fold2.pth", map_location=torch.device('cpu')))
    model.eval()
    print("Model & scaler loaded")

    # Start WS server
    Thread(target=lambda: asyncio.run(start_ws_server()), daemon=True).start()
    print("WebSocket server running in background.")

    try:
        print("tried")
        target_dt = 1.0 / 50

        t1 = Thread(target=get_prediction, args=(model,), daemon=True)
        t1.start()

        start_ts = time.perf_counter()
        screen_limit = 25
        count =0
        while True:
            if not streaming_event.is_set():
                time.sleep(0.01)
                start_ts = time.perf_counter()
                count = 0
                continue
            count +=1
            elapsedtime= time.perf_counter()-start_ts
            loop_start = time.perf_counter()
            next_call = start_ts + count * target_dt
            sleep_for = next_call - time.perf_counter()
            if sleep_for > 0:
                time.sleep(sleep_for)
            
            CombineData()
            combinecounter+=1

            if combinecounter> screen_limit and predicted_event.is_set() and len(buffer)>=50:
                combinecounter =1
                predicted_event.clear()
                print(f"{elapsedtime}")
        
    except KeyboardInterrupt:
        # If user presses Ctrl+C during the timer, on_exit will run
        pass
    finally:
        streaming_event.clear()
        stop_subscriptions()
        disconnect_sensors()
        print("All done. Exiting.")
        sys.exit(0)