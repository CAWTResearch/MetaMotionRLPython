from googleapiclient.discovery import build
from google.oauth2 import service_account

import os
import sys

SCOPES = ['https://www.googleapis.com/auth/drive']
SERVICE_ACCOUNT_FILE = 'service_account.json'
PARENT_FOLDER_ID = '16DwleohuGulUcZ0tjZHkda0lFqkJ6e7l'

# Root directory containing CSVs (one level up)
BASE_DIR = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))

# Ensure AccurateStreaming module is importable if needed
if BASE_DIR not in sys.path:
    sys.path.insert(0, BASE_DIR)

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

    # only upload these specific files
    for filename in ('combined_data.csv', 'predictions.csv'):
        path = os.path.join(BASE_DIR, filename)
        if os.path.isfile(path):
            upload_file(service, path, subfolder_id)
        else:
            print(f"[SKIP] {filename} not found in {BASE_DIR}")


if __name__ == '__main__':
    print_devices()
    upload_all_files()
    print('Files uploaded successfully.')
    print('See Google Drive for the uploaded files.')