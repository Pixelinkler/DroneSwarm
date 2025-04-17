import paramiko
import threading
import os
import time
from dronekit import connect

import math

# === Collision Avoidance Constants ===
DISTANCE_THRESHOLD_METERS = 3.0
POLL_INTERVAL = 0.5  # seconds

drone_sessions = {}
ssh_clients = {}  # key = drone IP, value = SSHClient object

# === Configuration ===
DRONES = [
    {"ip": "192.168.0.21", "user": "pi", "password": "pi"},
    {"ip": "192.168.0.20", "user": "pi", "password": "pi"}
]

LOCAL_CSV_FOLDER = "csv/"
REMOTE_FOLDER = "/home/pi/Drone_Swarm/Drone/"
REMOTE_MISSION_SCRIPT = REMOTE_FOLDER + "main.py"


def get_average_heading():
    headings = []

    def fetch_heading(ip, user, password, results, idx):
        try:
            print(f"[{ip}] Fetching heading...")
            client = paramiko.SSHClient()
            client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            client.connect(ip, username=user, password=password)
            ssh_clients[ip] = client

            actual_python = "/usr/local/opt/python-3.9.6/bin/python3.9"  # replace with full path if needed
            command = f"""
{actual_python} -c \"
import time
from dronekit import connect
vehicle = connect('127.0.0.1:14550', wait_ready=True)
while vehicle.gps_0.fix_type < 3:
    time.sleep(1)
print(vehicle.heading)
\"
"""
            stdin, stdout, stderr = client.exec_command(command)
            output = stdout.read().decode().strip()
            error = stderr.read().decode().strip()

            if output:
                heading = float(output.splitlines()[-1])
                print(f"[{ip}] Heading: {heading}°")
                results[idx] = heading
            elif error:
                print(f"[{ip}] Error: {error}")
                results[idx] = None

            
        except Exception as e:
            print(f"[{ip}] Exception: {str(e)}")
            results[idx] = None

    results = [None] * len(DRONES)
    threads = []

    for idx, drone in enumerate(DRONES):
        t = threading.Thread(target=fetch_heading, args=(drone["ip"], drone["user"], drone["password"], results, idx))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    valid_headings = [h for h in results if h is not None]

    if not valid_headings:
        print("No valid headings found.")
        return None

    avg_heading = sum(valid_headings) / len(valid_headings)
    print(f"Average Heading: {avg_heading:.2f}°")
    return avg_heading


def deploy_and_run(ip, user, password):
    try:
        print(f"[{ip}] Connecting via SSH...")

        client = ssh_clients.get(ip)
        if not client:
            print(f"[{ip}] No active SSH client available for deploy_and_run.")
            return

        sftp = client.open_sftp()

        # Ensure mission folder exists
        try:
            sftp.chdir(REMOTE_FOLDER)
        except IOError:
            print("Folder not found")
            return

        # Remove existing CSVs
        for file in sftp.listdir(REMOTE_FOLDER):
            if file.endswith(".csv"):
                sftp.remove(REMOTE_FOLDER + file)
                print(f"[{ip}] Removed old {file}")

        # Upload all CSVs from local CSV_FOLDER
        print(f"[{ip}] Uploading mission CSVs...")
        for file in os.listdir(LOCAL_CSV_FOLDER):
            if file.endswith(".csv"):
                local_path = os.path.join(LOCAL_CSV_FOLDER, file)
                remote_path = REMOTE_FOLDER + file
                sftp.put(local_path, remote_path)
                print(f"[{ip}] Uploaded {file}")

        sftp.close()
    except Exception as e:
        print(f"[{ip}] Exception during deployment: {str(e)}")


def run_main(ip, user, password, avg_initial_heading_deg):
    try:
        print(f"[{ip}] Again connecting via SSH...")
        client = ssh_clients.get(ip)
        if not client:
            print(f"[{ip}] No active SSH client available for run_main.")
            return

        print(f"[{ip}] Executing mission script...")
        print(f"[{ip}] Press button to start the show")

        stdin, stdout, stderr = client.exec_command(
            f"/usr/local/opt/python-3.9.6/bin/python3.9 {REMOTE_MISSION_SCRIPT} {avg_initial_heading_deg}"
        )

        for line in stdout:
            print(f"[{ip}] {line.strip()}")

        err = stderr.read().decode()
        if err:
            print(f"[{ip}] ERROR: {err.strip()}")

        print(f"[{ip}] Done.")

    except Exception as e:
        print(f"[{ip}] Exception: {str(e)}")

# === Main Launcher ===
def launch_all():
    avg_initial_heading_deg = get_average_heading()
    if avg_initial_heading_deg is None:
        print("Aborting: Couldn't determine average heading.")
        return

    if not os.path.exists(LOCAL_CSV_FOLDER):
        print("Error: main.py or mission.csv not found.")
        return

    threads = []
    for drone in DRONES:
        t = threading.Thread(
            target=deploy_and_run,
            args=(drone["ip"], drone["user"], drone["password"])
        )
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    print("CSVs uploaded in all drones!")    

    time.sleep(1)
    start = input("Enter 's' to start: ")
    if start == 's':
    
        threads = []
        for drone in DRONES:
            t = threading.Thread(
                target=run_main,
                args=(drone["ip"], drone["user"], drone["password"], avg_initial_heading_deg)
            )
            t.start()
            threads.append(t)

        for t in threads:
            t.join()    

    print("All drones deployed and missions initiated.")

if __name__ == "__main__":
    launch_all()
