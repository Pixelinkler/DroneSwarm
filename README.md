# DroneSwarm

This repository contains the complete setup for controlling a **swarm of drones** using **DroneKit** and **MAVLink** libraries in Python.

The system has two main components:
- **GCS (Ground Control Station)**
- **Drone Node (Raspberry Pi on each drone)**

Each drone executes its mission from a CSV file, while the GCS monitors helps initiate mission start.

## Project Structure

```
├── GCS/
│   ├── gcs_main.py
│   ├── utils/        # Helper scripts
│   └── csv/          # Mission CSVs for each drone
│
├── Drone/
│   ├── drone_main.py
│
├── README.md
└── requirements.txt
```

---

## 1. GCS (Ground Control Station)

- Communicates with all drones via MAVLink.
- Sends commands, receives telemetry, and monitors status.
- Designed to run on a **laptop/PC** with MAVLink (WiFi) communication setup.
- Uploads CSV mission files from the `csv/` folder to each drone using **UDP** connection.

### Key Features
- Connect to multiple drones simultaneously.
- Send swarm commands like **Takeoff**, **Land**, etc.
- Extendable for future functionalities (e.g., **formation flying**, **dynamic tasking**).

### Drone List Configuration

Inside `gcs_main.py`, define the list of drones:

```python
DRONES = [
    {"ip": "192.168.0.21", "user": "pi", "password": "pi"},
    {"ip": "192.168.0.20", "user": "pi", "password": "pi"}
]
```

Each element contains:
- `ip`: IP address of the drone’s Raspberry Pi
- `user`: SSH username (default: `pi`)
- `password`: SSH password

> **Note:**  
> The list order determines the **drone numbering**, which affects **initial drone positioning** in missions.

---

## 2. Drone (Raspberry Pi)

- Script (`main.py`) runs individually on each drone’s Raspberry Pi.
- Automatically initiated by GCS.
- Reads a **mission.csv** containing waypoints and parameters.
- Executes the mission autonomously.

### Key Features
- Local execution of CSV-based missions.
- Communication with GCS via MAVLink.
- Future scope:
  - LED control (e.g., mission-based lighting)
  - Drone collision avoidance

---

## 3. Mission Planning Workflow

We use **Skybrush** (a Blender plugin) for designing and simulating swarm shows.

### Workflow Steps:
1. **Design** the swarm movements using Blender + Skybrush.
2. **Simulate and validate** the show within Blender.
3. **Export** individual mission CSV files for each drone and place them inside the `GCS/csv/` folder.

Each drone independently follows its assigned CSV mission.

> **Safety Feature:**  
> Each drone is equipped with an **ELRS receiver** connected to a **common transmitter** with switches for:
> - Arm
> - Land
> - Position Hold
>
> This allows immediate manual override for all drones during emergencies (e.g., crash, GPS glitch).

---

## 4. Drone Raspberry Pi Setup

- Install a **lightweight server image** on Raspberry Pi Zero 2 W.
- Setup **mavlink-router** for UDP communication:
  - GitHub: [mavlink-router](https://github.com/mavlink-router/mavlink-router)
- Install Python 3.9 libraries:
  ```bash
  sudo pip install dronekit pymavlink
  ```
- Add the **mavrouter connection command** to `.bashrc` for auto-start on boot.

---

## 5. Hardware Setup

| Component           | Model / Description                              |
| ------------------- | ------------------------------------------------- |
| Flight Controller   | Speedybee F405 / Pixhawk 2.4.8 (ArduPilot-supported) |
| ESC                 | 4-in-1 ESC (compatible with 5-inch frames)        |
| Frame               | 5-inch Drone Frame                                |
| Receiver            | ELRS (ExpressLRS) Receivers                       |
| Transmitter         | ELRS Compatible Radio Transmitter                 |
| Motors              | Brushless Motors (for 5-inch frames)              |
| Onboard Computer    | Raspberry Pi Zero 2 W                             |
| GPS / Compass       | M9 Micro GPS Module                               |
| Propellers          | 5-inch Propellers                                 |
| Battery             | Li-ion 4500mAh 4S Pack                            |
| LED Lighting        | NeoPixel LED Strip or Ring (Addressable RGB)      |

### Notes:
- **Flight Controllers** must run **ArduPilot Copter firmware** for DroneKit compatibility.
- **NeoPixel LEDs** can be programmed for synchronized light shows.
- **ELRS system** allows low-latency, long-range manual control over all drones.
- **Raspberry Pi Zero 2 W** handles autonomous mission control + optional LED triggers.
- **M9 Micro GPS** ensures precise positioning for formation flights.
- **Li-ion batteries** are used for longer endurance — ideal for swarm shows.


# Happy Flying! 🚁✨
