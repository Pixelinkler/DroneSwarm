# DroneSwarm
This repository contains the complete setup for controlling a swarm of drones using the DroneKit and MAVLink libraries in Python.
The system is designed with two main components:

    GCS (Ground Control Station)

    Drone Node (Raspberry Pi on each drone)

Each drone executes its mission from a CSV file, while the GCS monitors and manages the entire swarm.

├── GCS/
│   ├── gcs_main.py
│   └── utils/ (helper scripts)
|   └── csv/ (mission.csv for each drone)
│
├── Drone/
│   ├── drone_main.py
│
├── README.md
└── requirements.txt

1. GCS (Ground Control Station)

    This is the main control node that communicates with all drones via MAVLink.
    It sends commands, receives telemetry, and monitors the status of each drone.
    Designed to run on a laptop/PC with a MAVLink communication setup (WiFi)
    This also upload csv file that are in the csv folder to each drone using udp connection
   
Key Features:

    Connection to multiple drones simultaneously.

    Ability to send swarm commands like Takeoff, Land.

    Extendable for future functionalities (formation flying, dynamic tasking, etc).

# Drone List Configuration

Inside the GCS script (gcs.py), you must define the list of drones you want to control.
Example:

DRONES = [
    {"ip": "192.168.0.21", "user": "pi", "password": "pi"},
    {"ip": "192.168.0.20", "user": "pi", "password": "pi"}
]

Each dictionary contains:
    ip: IP address of the drone’s Raspberry Pi.
    user: SSH username (default pi for Raspberry Pi OS).
    password: SSH password.
    
And also don't forget that this is also the numbering order of each drone which mean 
it will also decide where each drone will be kept at the start of the mission.


2. Drone (Raspberry Pi on Each Drone)

    This script runs individually on each drone's Raspberry Pi.
    This script is automatically initiated by the GCS
    It reads a mission.csv file that contains the list of waypoints and mission parameters.
    The drone autonomously follows its mission once started.

Key Features:

    Local execution of missions based on a CSV plan.

    Communication with GCS via MAVLink.

    Future scope to add LED control,Drone collision Avoidance

3.Mission Planning Workflow

We use Skybrush — a plugin in Blender — to design and simulate the swarm shows.
Once the choreography is finalized, Skybrush exports a CSV file for each drone containing its specific mission path.

    Step 1: Design swarm movement using Blender and the Skybrush plugin.

    Step2: Simulate and validate the show within Blender.

    Step3: Export individual mission CSV files for each drone and add it in the csv folder in GCS.

Each drone follows its own CSV mission independently. 

For Safety every Drone is equiped with an ELRS reciver,these all receviers are connected to one transmitter.
This transmitter has switches for arm, land, position hold modes and every channel is common in all the drones so 
we can land all the drones in case of crashed and GPS gliching.


4. Drone Raspberry Pi Setup

Install a light weight server image for raspberry pi.
Setup mavrouter for udp connection to the GCS and QGC : https://github.com/mavlink-router/mavlink-router
Also setup python 3.9 for dronekit : sudo pip install dronekit pymavlink.
Add the connection command of the mavrouter to bashrc file to start it as soon as the pi turns on.

5. Hardware Setup

Each drone is built with the following hardware components:

Component | Model / Description

Flight Controller | Speedybee F405 / Pixhawk 2.4.8 (ArduPilot-supported)
ESC (Electronic Speed Controller) | 4-in-1 ESC (compatible with 5-inch frames)
Frame | 5-inch Drone Frame
Receiver | ELRS (ExpressLRS) Receivers
Transmitter | ELRS Compatible Radio Transmitter
Motors | Brushless Motors (suited for 5-inch frames)
Onboard Computer | Raspberry Pi Zero 2 W
GPS / Compass | M9 Micro GPS Module
Propellers | 5-inch Propellers
Battery | Li-ion 4500mAh 4S Pack
LED Lighting | NeoPixel LED Strip or Ring (Addressable RGB)

Notes:

    Flight Controllers must run ArduPilot Copter firmware for compatibility with DroneKit and MAVLink.
    NeoPixel LEDs can be connected to the Raspberry Pi and programmed for synchronized light shows.
    ELRS provides low-latency, long-range control if manual override is needed.Plus added benefit of connecting multiple drones to one transmitter
    Raspberry Pi Zero 2 W handles mission control and can also trigger LED animations based on mission events.
    M9 Micro GPS ensures precise positioning, which is critical for formation flying.
    Li-ion Batteries are preferred for longer endurance compared to LiPo, important for swarm shows.




