#Script to test GPS working

from dronekit import connect
import time

# Connect to the vehicle (assumes SITL or MAVProxy running at this endpoint)
vehicle = connect('127.0.0.1:14550', wait_ready=True)




while True:
    while vehicle.gps_0.fix_type < 3:
        time.sleep(1)
    loc = vehicle.location.global_relative_frame
    print(f'{loc.lat},{loc.lon}', flush=True)
    time.sleep(0.5)
