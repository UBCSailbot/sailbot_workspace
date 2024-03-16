import os
import pickle
import zipfile
from shutil import move

import gdown
import requests
from tqdm import tqdm

DOWNLOAD_ATTEMPTS = 3


class colors:
    """
    ANSI color codes for terminal output.
    """

    # I should have just created a logger to use instead of this
    ERROR = "\033[91m"
    OK = "\033[92m"
    WARN = "\033[93m"
    RESET = "\033[0m"


def dump_pkl(object: any, file_path: str):
    # creating a pickler once, when everything is being pickled
    # will likely be more efficient
    with open(file_path, "wb") as f:
        pickle.dump(object, f, protocol=pickle.HIGHEST_PROTOCOL)


def load_pkl(file_path: str) -> any:
    with open(file_path, "rb") as f:
        return pickle.load(f)


def download_zip(url: str, file_name: str, dir: str):
    """
    Saves a file from a URL to the specified directory with the specified name.
    Then calls the unzip() function.

    Args:
        - url (str): The URL of the file to be downloaded.
        - file_name (str): The file name to save the downloaded file to.
        - dir (str): The directory to save the downloaded file to.
    """
    tries = DOWNLOAD_ATTEMPTS

    while tries > 0:

        try:
            # download file in chunks of 10MB
            response = requests.get(
                url, stream=True
            )  # stream to avoid loading entire file into memory
            path = os.path.join(dir, file_name)
            with open(path, "wb") as f:
                for chunk in tqdm(
                    response.iter_content(chunk_size=10 * 1024**2),
                    desc=f"Downloading {file_name} to {dir}...",
                    total=int(int(response.headers.get("content-length", 0)) / (10 * 1024**2) + 1),
                ):
                    f.write(chunk)
            # extract files to shp folder
            print(f"Extracting {file_name} to {dir}...")
            unzip(path, extract_to=dir)
            break

        except requests.exceptions.RequestException as e:
            print(colors.ERROR + f"Failed to download {file_name} from {url}." + colors.RESET)
            print(e)
            print("Retrying...")
            tries -= 1
            if tries == 0:
                exit(e)


def flatten_dir(directory):
    """
    Flatten a directory structure by moving all files to the parent directory and then removing
    any subdirectories

    Args:
        - directory (str): The directory to be flattened.
    """
    for root, dirs, files in os.walk(directory):
        # Move all files to the parent directory
        for file in files:
            src = os.path.join(root, file)
            dst = os.path.join(directory, file)
            move(src, dst)
    for root, dirs, files in os.walk(directory, topdown=False):
        for dir in dirs:
            os.rmdir(os.path.join(root, dir))


def gd_download(url: str, file_name: str):

    tries = DOWNLOAD_ATTEMPTS
    while tries > 0:
        try:
            gdown.download(url, file_name, quiet=False)
            break
        except gdown.exceptions.FileURLRetrievalError:
            print(colors.ERROR + f"Failed to download {file_name} from {url}." + colors.RESET)
            print("Retrying...")
            tries -= 1


def unzip(zip_file, extract_to):
    """
    Extracts the contents of a zip file to a specified directory,
    then deletes the original zip file.

    Args:
        - zip_file (str): The file path of the zip file to be unzipped.
        - extract_to (str): The directory to unzip the file to.
    """
    with zipfile.ZipFile(zip_file, "r") as zip_ref:
        zip_ref.extractall(extract_to)
    os.remove(zip_file)
