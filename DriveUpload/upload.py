from googleapiclient.discovery import build
from google.oauth2 import service_account

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

def create_subfolder(service, name, parent_id):
    file_metadata = {
        'name': name,
        'mimeType': 'application/vnd.google-apps.folder',
        'parents': [parent_id]
    }
    folder = service.files().create(body=file_metadata, fields='id').execute()
    return folder.get('id')

def upload_photo(service, file_path, folder_id):
    file_metadata = {
        'name': os.path.basename(file_path),
        'parents': [folder_id]
    }

    file = service.files().create(
        body=file_metadata,
        media_body=file_path
    ).execute()

def print_devices():
    print("Sensors:", device_macs)
    print("Dongles:", dongle_macs)

def upload_all_files():
    creds = authenticate()
    service = build('drive', 'v3', credentials=creds)

    # Create a subfolder with a timestamp or custom name
    import datetime
    subfolder_name = "Upload_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    subfolder_id = create_subfolder(service, subfolder_name, PARENT_FOLDER_ID)

    # Upload files to the new subfolder
    upload_photo(service, 'combined_data.csv', subfolder_id)
    upload_photo(service, 'predictions.csv', subfolder_id)

if __name__ == "__main__":
    print_devices()
    upload_all_files()
    print("Files uploaded successfully.")
    print("See Google Drive for the uploaded files.")