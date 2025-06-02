#pip install google-api-python-client
# from googleapiclient.discovery import build
# from google.oauth2 import service_account

import os
import sys

SCOPES = ['https://www.googleapis.com/auth/drive']
SERVICE_ACCOUNT_FILE = 'service_account.json'
PARENT_FOLDER_ID = "16DwleohuGulUcZ0tjZHkda0lFqkJ6e7l"

# 1) Compute the absolute path one level up (MetaMotionRLPython)
parent = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))

# 2) Insert it at the front of sys.path so Python can find AccurateStreaming.py
if parent not in sys.path:
    sys.path.insert(0, parent)

# 3) Now do a normal (absolute) import
from AccurateStreaming import device_macs, dongle_macs

def authenticate():
    creds = service_account.Credentials.from_service_account_file(SERVICE_ACCOUNT_FILE, scopes=SCOPES)
    return creds

def upload_photo(file_path):
    creds = authenticate()
    service = build('drive', 'v3', credentials=creds)

    file_metadata = {
        'name' : file_path,
        'parents' : [PARENT_FOLDER_ID]
    }

    file = service.files().create(
        body=file_metadata,
        media_body=file_path
    ).execute()

def print_devices():
    print("Sensors:", device_macs)
    print("Dongles:", dongle_macs)

if __name__ == "__main__":
    print_devices()

def upload_all_files():
    for device in device_macs:
        upload_photo('acc_' + device + '.csv')
        upload_photo('gyro_' + device + '.csv')

upload_all_files()
print("File uploaded successfully.")
print('See Google Drive for the uploaded files')