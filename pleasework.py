from __future__ import print_function
from mbientlab.metawear import MetaWear, libmetawear, parse_value, create_voidp
from mbientlab.metawear.cbindings import AccBmi270Odr, AccBoschRange, GyroBoschRange, \
GyroBoschOdr, FnVoid_VoidP_DataP, FnVoid_VoidP_UInt_UInt, LogDownloadHandler, cast, byref, \
FnVoid_VoidP_UByte_Long_UByteP_UByte, SensorFusionData
from mbientlab.warble import BleScanner
from time import sleep
from threading import Event
from datetime import datetime
import os
import time
from types import SimpleNamespace


# Get time and date
now = datetime.now()
stamp = f'{now.year}{now.month}{now.day}_{now.hour}-{now.minute}-{now.second}'

MAC_devices = ["F7:68:55:8D:84:0E"]

os.makedirs('logs', exist_ok=True)
data_downloaders = [None] * len(MAC_devices)

input_arr = list(enumerate(MAC_devices))

# Open file lists
acc_log_list = [None]*len(MAC_devices)
acc_stream_list = [None]*len(MAC_devices)
gyro_log_list = [None]*len(MAC_devices)
gyro_stream_list = [None]*len(MAC_devices)

# Logger list
device_loggers = [0]*len(MAC_devices)

# State list
device_states = [None]*len(MAC_devices)

# Device list
device_instances = [None]*len(MAC_devices)

# Prepare the log lists
for i, device in enumerate(MAC_devices):
    acc_log_list[i] = open(f'logs/acc_log_{i+1}_{stamp}.txt', 'w')
    gyro_log_list[i] = open(f'logs/gyro_log_{i+1}_{stamp}.txt', 'w')


class DataDownloader:
    def __init__(self, state, loggers, index):
        self.entries_left = 100 #Placeholder
        self.state = state
        self.device = state.device
        self.index = index
        self.loggers = loggers

        self.callback_acc = FnVoid_VoidP_DataP(lambda ctx, p: self.data_logger_acc(p))
        self.callback_gyro = FnVoid_VoidP_DataP(lambda ctx, p: self.data_logger_gyro(p))

        self.fn_wrapper = FnVoid_VoidP_UInt_UInt(self.progress_update_handler)
        self.download_handler = LogDownloadHandler(context = None,
                        received_progress_update = self.fn_wrapper,
                        received_unknown_entry = cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
                        received_unhandled_entry = cast(None, FnVoid_VoidP_DataP))

        #time_start = time.perf_counter()
        libmetawear.mbl_mw_logger_subscribe(self.loggers[0], None, self.callback_acc)
        libmetawear.mbl_mw_logger_subscribe(self.loggers[1], None, self.callback_gyro)
        libmetawear.mbl_mw_logging_download(self.device.board, 0,
                                            byref(self.download_handler))

    def progress_update_handler(self, context, entries_left, total_entries):
        self.entries_left = entries_left

    def data_logger_acc(self, p):
        acc_log_list[self.index].write(f'{p.contents.epoch}, {parse_value(p)}\n')

    def data_logger_gyro(self, p):
        gyro_log_list[self.index].write(f'{p.contents.epoch}, {parse_value(p)}\n')

def setup_device(state):

    libmetawear.mbl_mw_acc_bmi270_set_odr(state.device.board, AccBmi270Odr._100Hz)
    libmetawear.mbl_mw_acc_bosch_set_range(state.device.board, AccBoschRange._16G)
    libmetawear.mbl_mw_acc_write_acceleration_config(state.device.board)


    libmetawear.mbl_mw_gyro_bmi270_set_range(state.device.board, GyroBoschRange._500dps)
    libmetawear.mbl_mw_gyro_bmi270_set_odr(state.device.board, GyroBoschOdr._100Hz)
    libmetawear.mbl_mw_gyro_bmi270_write_config(state.device.board)


    acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(state.device.board)

    acc_logger = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(acc, None, fn), resource = "acc_logger")


    gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(state.device.board)

    gyro_logger = create_voidp(lambda fn: libmetawear.mbl_mw_datasignal_log(gyro, None, fn), resource = "gyro_logger")
    return (acc_logger, gyro_logger)

def start_device(state):
    libmetawear.mbl_mw_logging_start(state.device.board, 0)

    libmetawear.mbl_mw_acc_enable_acceleration_sampling(state.device.board)
    libmetawear.mbl_mw_acc_start(state.device.board)

    libmetawear.mbl_mw_gyro_bmi270_enable_rotation_sampling(state.device.board)
    libmetawear.mbl_mw_gyro_bmi270_start(state.device.board)
    return

def stop_device(state):
    libmetawear.mbl_mw_logging_stop(state.device.board)
    libmetawear.mbl_mw_logging_flush_page(state.device.board)
    sleep(0.5)    # give it a moment to write out the last page

    libmetawear.mbl_mw_acc_stop(state.device.board)
    libmetawear.mbl_mw_acc_disable_acceleration_sampling(state.device.board)
    libmetawear.mbl_mw_gyro_bmi270_stop(state.device.board)
    libmetawear.mbl_mw_gyro_bmi270_disable_rotation_sampling(state.device.board)

    acc = libmetawear.mbl_mw_acc_get_acceleration_data_signal(state.device.board)
    libmetawear.mbl_mw_datasignal_unsubscribe(acc)
    gyro = libmetawear.mbl_mw_gyro_bmi270_get_rotation_data_signal(state.device.board)
    libmetawear.mbl_mw_datasignal_unsubscribe(gyro)

    libmetawear.mbl_mw_logging_flush_page(state.device.board)
    return

def download_data(state, loggers, index):
    e = Event()

    device = state.device

    def progress_update_handler(context, entries_left, total_entries):
        if (entries_left == 0):
            e.set()

    def data_logger_acc(p):
        acc_log_list[index].write(f'{p.contents.epoch}, {parse_value(p)}\n')

    def data_logger_gyro(p):
        gyro_log_list[index].write(f'{p.contents.epoch}, {parse_value(p)}\n')

    callback_acc = FnVoid_VoidP_DataP(lambda ctx, p: data_logger_acc(p))
    callback_gyro = FnVoid_VoidP_DataP(lambda ctx, p: data_logger_gyro(p))

    fn_wrapper = FnVoid_VoidP_UInt_UInt(progress_update_handler)
    download_handler = LogDownloadHandler(context = None,
                    received_progress_update = fn_wrapper,
                    received_unknown_entry = cast(None, FnVoid_VoidP_UByte_Long_UByteP_UByte),
                    received_unhandled_entry = cast(None, FnVoid_VoidP_DataP))

    libmetawear.mbl_mw_logger_subscribe(loggers[0], None, callback_acc)
    libmetawear.mbl_mw_logger_subscribe(loggers[1], None, callback_gyro)
    libmetawear.mbl_mw_logging_download(device.board, 0,
                                        byref(download_handler))
    e.wait()

    e.clear()
    return True

def start(MAC_devices):
    for index, MAC in enumerate(MAC_devices):
        device = MetaWear(MAC)
        device.connect()
        libmetawear.mbl_mw_settings_set_connection_parameters(device.board, 7.5, 7.5, 0, 6000)
        sleep(1.0)

        state = SimpleNamespace(device=device) 

        loggers = setup_device(state)
        state = DataDownloader(state, loggers, index)

        start_device(state)
        device_loggers[index] = loggers

        device_states[index] = state

        device_instances[index] = device
    return

def stop(MAC_devices):
    # Connect back to devices
    for index, device in enumerate(device_instances):
        device.connect()
        print(f'Connected to {device.address}, device number {index+1}.')

    # Stops the whole thing, downloads the data
    for index, state in enumerate(device_states):
        # Stop logging for this device
        print(f'Stopping device number {index+1}.')
        stop_device(state)

    # Start downloading the data
    for index, state in enumerate(device_states):
        # Download data
        print(f'Downloading data for device number {index+1}.')
        data_downloaders[index] = DataDownloader(state, device_loggers[index], index)

    downloading = True
    time0 = time.time()
    while downloading:
        # First, get the remaining entries left
        entries_left_list = [x.entries_left for x in data_downloaders]

        # Print it cuz why not
        device_status = ['Ready' if x == 0 else 'Downloading' for x in entries_left_list ]
        print('--------------------------------------------')
        print(f'Status of devices: {device_status}')
        print('Time elapsed: {int(time.time()-time0)} seconds.')

        # Check if all zero
        if all([x==0 for x in entries_left_list]):
            # We're done
            print('Download complete!')
            downloading = False

        # Sleep a bit
        if downloading:
            sleep(10)

    for index, state in enumerate(device_states):    
        acc_log_list[index].close()
        gyro_log_list[index].close()
        print(f'Logs saved, disconnecting device number {index+1}.')
        libmetawear.mbl_mw_debug_disconnect(state.device.board)

    return

def reset_dev(MAC):
    device = MetaWear(MAC)
    device.connect()

    libmetawear.mbl_mw_logging_stop(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_logging_flush_page(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_logging_clear_entries(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_event_remove_all(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_macro_erase_all(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_debug_reset_after_gc(device.board)
    sleep(1.0)

    libmetawear.mbl_mw_debug_disconnect(device.board)
    sleep(1.0)

    device.disconnect()
    sleep(1.0)

def reset_devices(MAC_devices):
    for MAC in MAC_devices:
        reset_dev(MAC)

def main():
    devices = []
    try:
        # 1) reset & start all devices
        reset_devices(MAC_devices)
        for mac in MAC_devices:
            d = MetaWear(mac)
            d.connect()
            sleep(2.0)                            # give the stack a moment
            devices.append(d)
            state = SimpleNamespace(device=d)
            loggers = setup_device(state)
            downloader = DataDownloader(state, loggers, MAC_devices.index(mac))
            start_device(state)
        print("Logging... Ctrl+C to stop.")
        while True:
            sleep(1)

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received — stopping devices…")

    finally:
        # 2) stop logging on each board
        for d in devices:
            try:
                libmetawear.mbl_mw_logging_stop(d.board)
                sleep(0.5)
            except Exception:
                pass

        # 3) unsubscribe & flush & download
        for idx, d in enumerate(devices):
            try:
                stop_device(device_states[idx])
                download_data(device_states[idx], device_loggers[idx], idx)
                sleep(0.5)
            except Exception:
                pass

        # 4) fully disconnect each board
        for d in devices:
            try:
                # debug_disconnect forces the on-board CPU to close the link
                libmetawear.mbl_mw_debug_disconnect(d.board)
                sleep(1.0)                   # << critical!
                d.disconnect()
                sleep(1.0)
            except Exception as e:
                # catch the BLEPP::SocketConnectFailed or busy errors
                print(f"Warning disconnecting {d.address}: {e}")

        print("Clean shutdown complete.")


if __name__ == '__main__':
    main()