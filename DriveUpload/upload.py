from googleapiclient.discovery import build
from google.oauth2 import service_account

import os
import sys

SCOPES = ['https://www.googleapis.com/auth/drive']
SERVICE_ACCOUNT_FILE = 'service_account.json'
PARENT_FOLDER_ID = '16DwleohuGulUcZ0tjZHkda0lFqkJ6e7l'

# Directory to scan for CSVs: the folder containing this script
BASE_DIR = os.path.abspath(os.path.dirname(__file__))
print(f"Scanning directory: {BASE_DIR}")
print("Contents:", os.listdir(BASE_DIR))

# Ensure AccurateStreaming module is importable if needed
parent_dir = os.path.abspath(os.path.join(BASE_DIR, os.pardir))
if parent_dir not in sys.path:
    sys.path.insert(0, parent_dir)

from AccurateStreaming import device_macs, dongle_macs

def authenticate():
    creds = service_account.Credentials.from_service_account_file(
        SERVICE_ACCOUNT_FILE, scopes=SCOPES)
    return creds


def create_subfolder(service, name, parent_id):
    metadata = {
        'name': name,
        'mimeType': 'application/vnd.google-apps.folder',
        'parents': [parent_id]
    }
    folder = service.files().create(body=metadata, fields='id').execute()
    return folder.get('id')


def upload_file(service, file_path, folder_id):
    file_metadata = {
        'name': os.path.basename(file_path),
        'parents': [folder_id]
    }
    service.files().create(
        body=file_metadata,
        supportsAllDrives=True,
        media_body=file_path
    ).execute()


def print_devices():
    print("Sensors:", device_macs)
    print("Dongles:", dongle_macs)


def upload_all_files():
    creds = authenticate()
    service = build('drive', 'v3', credentials=creds)

    # create a timestamped subfolder
    import datetime
    subfolder = 'Upload_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    subfolder_id = create_subfolder(service, subfolder, PARENT_FOLDER_ID)

    # upload specific files
    for filename in ('combined_data.csv', 'predictions.csv'):
        path = os.path.join(BASE_DIR, filename)
        if os.path.isfile(path):
            print(f"Uploading {filename}")
            upload_file(service, path, subfolder_id)
        else:
            print(f"[SKIP] {filename} not found in {BASE_DIR}")


if __name__ == '__main__':
    print_devices()
    upload_all_files()
    print('Operation complete.')
