class DroneController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.setup_channel_listener()

    def setup_channel_listener(self):
        @self.vehicle.on_message('RC_CHANNELS')
        def ch5_listener(self, name, message):
            ch5_val = message.chan5_raw  # Equivalent to vehicle.channels['5']
            print(f"Ch5: {ch5_val}")

# Usage
from dronekit import connect
import time

vehicle = connect('127.0.0.1:14550', wait_ready=True)
controller = DroneController(vehicle)

try:
    while True:
        time.sleep(1)  # Just keep the program alive to allow message processing
except KeyboardInterrupt:
    print("Shutting down...")
finally:
    vehicle.close()

