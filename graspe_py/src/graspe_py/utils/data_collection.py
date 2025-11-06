import os
import csv
import time
import argparse
from datetime import datetime
from graspe_py.comms.serialBridge import SerialBridge

# Default: current working directory where the script is executed
DEFAULT_DATA_FOLDER = os.getcwd()

"""
    Argument parser configuration
"""
parser = argparse.ArgumentParser(description="Script for collecting motor data via SerialLink")

parser.add_argument(
    "--dir_path",
    type=str,
    default=DEFAULT_DATA_FOLDER,
    help="Path to the directory where data will be saved"
)

parser.add_argument(
    "--file_name",
    type=str,
    default=datetime.now().strftime("%Y_%m_%d_%H_%M_%S.csv"),
    help="Output CSV file name (default: timestamp)"
)

parser.add_argument(
    "--flush_interval",
    type=int,
    default=50,
    help="Number of samples before saving to disk (default: 50)"
)

parser.add_argument(
    "--port",
    type=str,
    default="/dev/ttyUSB0",
    help="Serial port to connect to"
)

args = parser.parse_args()
dir_path = args.dir_path
file_name = args.file_name
if not file_name.endswith(".csv"):
    file_name += ".csv"
flush_interval = args.flush_interval
serial_port = args.port

# Create directory if it doesn’t exist
os.makedirs(dir_path, exist_ok=True)
path_to_file = os.path.join(dir_path, file_name)

# Initialize CSV file with header
with open(path_to_file, mode="w", newline='', encoding="utf-8") as file:
    writer = csv.writer(file)
    writer.writerow(["timestamp", "q1", "q2", "q3", "q4"])
print(f"[INFO] File initialized at: {path_to_file}")

"""
    Serial communication setup
"""
print("[INFO] Performing handshake with ESP32...")
serial_bridge = SerialBridge(serial_port)

attempts = 0
while not serial_bridge.handshake(timeout=3.0):
    attempts += 1
    print(f"[WARN] Failed to handshake... trying again (attempt #{attempts})")
    time.sleep(1)
print("[INFO] Handshake successful")

"""
    Data collection loop
"""
print("[INFO] Collecting data... Press Ctrl+C to stop.")
dados = []
total_samples = 0

try:
    while True:
        data = serial_bridge.get_joints_position()
        if data:
            timestamp = time.time()
            dados.append([timestamp, *data])

            # Save periodically
            if len(dados) >= flush_interval:
                with open(path_to_file, mode="a", newline='', encoding="utf-8") as file:
                    writer = csv.writer(file)
                    writer.writerows(dados)
                total_samples += len(dados)
                print(f"[INFO] Saved {len(dados)} samples to disk. Total so far: {total_samples}")
                dados.clear()

except KeyboardInterrupt:
    print("\n[INFO] Interruption detected! Saving remaining data...")

    if dados:
        with open(path_to_file, mode="a", newline='', encoding="utf-8") as file:
            writer = csv.writer(file)
            writer.writerows(dados)
        total_samples += len(dados)

    print(f"[INFO] Data collection finished. File saved at: {path_to_file}")
    print(f"[INFO] Total samples recorded: {total_samples}")
