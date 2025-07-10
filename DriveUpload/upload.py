from googleapiclient.discovery import build
from google.oauth2 import service_account

import os
import sys

SCOPES = ['https://www.googleapis.com/auth/drive']
SERVICE_ACCOUNT_FILE = 'service_account.json'


ParentFolder = "RealTimeTesting Predictions"

folders_ID = {"Yahid": "1BVlVORstArc-x2uptACGK1vqFcks5SCW", 
              "Angel": "18KIELRL5BBtaBpIirm9wc1DhOnkM3W8B",
              "RealTimeTesting Predictions": "1gkEMBR54HxqMs806p1jvURIYh7wUjzwj",
              "CAWT_DATA": "16DwleohuGulUcZ0tjZHkda0lFqkJ6e7l"
              }

PARENT_FOLDER_ID = folders_ID[ParentFolder]

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

def upload_file(service, file_path, folder_id):
    name = input("Enter the name for the file: ")

    file_metadata = {
        'name': name,
        'parents': [folder_id]
    }

    file = service.files().create(
        body=file_metadata,
        media_body=file_path
    ).execute()


def upload_all_files():
    creds = authenticate()
    service = build('drive', 'v3', credentials=creds)

    # Create a subfolder with a timestamp or custom name
    subfolder_name = input("Enter the name for the subfolder: ")

    if not subfolder_name in folders_ID:
        subfolder_id = create_subfolder(service, subfolder_name, PARENT_FOLDER_ID)
        folders_ID[subfolder_name] = subfolder_id
    

    subfolder_id = folders_ID[subfolder_name]

    # Upload files to the new subfolder
    upload_file(service, 'predictions.csv', subfolder_id)

if __name__ == "__main__":
    upload_all_files()
    print("Files uploaded successfully.")
    print("See Google Drive for the uploaded files.")