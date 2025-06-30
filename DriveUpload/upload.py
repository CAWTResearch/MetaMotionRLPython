from googleapiclient.discovery import build
from google.oauth2 import service_account

import os
import sys

# Constants
SCOPES = ['https://www.googleapis.com/auth/drive']
SERVICE_ACCOUNT_FILE = 'service_account.json'
PARENT_FOLDER_ID = '16DwleohuGulUcZ0tjZHkda0lFqkJ6e7l'  # Folder in a Shared Drive

# 1) Compute the absolute path one level up (MetaMotionRLPython)
parent = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
# 2) Ensure Python can import AccurateStreaming from that folder
if parent not in sys.path:
    sys.path.insert(0, parent)
# 3) Import device and dongle lists
from AccurateStreaming import device_macs, dongle_macs


def authenticate():
    """
    Authenticate using a Service Account. Make sure the service account
    has Content Manager access to the Shared Drive containing PARENT_FOLDER_ID.
    """
    creds = service_account.Credentials.from_service_account_file(
        SERVICE_ACCOUNT_FILE,
        scopes=SCOPES
    )
    return creds


def create_subfolder(service, name, parent_id):
    """
    Create a timestamped folder under the given parent in a Shared Drive.
    Requires supportsAllDrives=True.
    """
    metadata = {
        'name': name,
        'mimeType': 'application/vnd.google-apps.folder',
        'parents': [parent_id]
    }
    folder = service.files().create(
        body=metadata,
        supportsAllDrives=True,
        fields='id'
    ).execute()
    return folder.get('id')


def upload_file(service, file_path, folder_id):
    """
    Upload a local file to the specified folder in the Shared Drive.
    """
    metadata = {
        'name': os.path.basename(file_path),
        'parents': [folder_id]
    }
    service.files().create(
        body=metadata,
        media_body=file_path,
        supportsAllDrives=True,
        fields='id'
    ).execute()


def print_devices():
    print('Sensors:', device_macs)
    print('Dongles:', dongle_macs)


def upload_all_files():
    # Authenticate & build Drive service
    creds = authenticate()
    service = build('drive', 'v3', credentials=creds)

    # Create a new subfolder
    import datetime
    subfolder_name = 'Upload_' + datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
    subfolder_id = create_subfolder(service, subfolder_name, PARENT_FOLDER_ID)

    # Only upload these two files
    files_to_upload = ['combined_data.csv', 'predictions.csv']
    for name in files_to_upload:
        path = os.path.join(parent, name)
        if os.path.exists(path):
            upload_file(service, path, subfolder_id)
            print(f"Uploaded: {name}")
        else:
            print(f"Missing file: {path}")


if __name__ == '__main__':
    print_devices()
    upload_all_files()
    print('Done! Files are in your Shared Drive.')
