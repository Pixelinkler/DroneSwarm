from dronekit import connect, VehicleMode, LocationGlobalRelative
import csv
import time
import math
from pymavlink import mavutil
import sys

connection_string = '127.0.0.1:14550' 
vehicle = connect(connection_string, wait_ready=True)
hello=1
class DroneController:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.cha5=None
        self.setup_channel_listener()

    def setup_channel_listener(self):
        @self.vehicle.on_message('RC_CHANNELS')
        def ch5_listener(self, name, message):
            global hello 
            hello= message.chan5_raw
            return message.chan5_raw


def gps_heading(vehicle):   
    while vehicle.gps_0.fix_type < 3:
        time.sleep(0.1)
    heading = vehicle.heading
    return heading

def read_waypoints(csv_file):
    
    waypoints = []
    last_x, last_y, last_z = None, None, None
    x0, y0 = None, None
    timestamps = []
    rotated_coords = []
    z_values = []

    initial_heading_deg = float(sys.argv[1]) if len(sys.argv) > 1 else gps_heading(vehicle)
    theta = math.radians(initial_heading_deg)

    with open(csv_file, newline='') as file:
        reader = csv.reader(file)
        next(reader)  # Skip header

        for idx, row in enumerate(reader):
            time_stamp, x, y, z, red, green, blue = map(float, row)

            if idx == 0:
                x0, y0 = x, y  # Set the origin

            # Shift to make first row the origin
            x_rel = x - x0
            y_rel = -1*(y - y0)

            # Rotate relative coordinates using initial heading
            x_rot = round(x_rel * math.cos(theta) - y_rel * math.sin(theta), 7)
            y_rot = round(x_rel * math.sin(theta) + y_rel * math.cos(theta), 7)
            z = round(z, 7)  # Round Z to avoid floating point noise

            # Check for duplicates
            if not rotated_coords or (x_rot, y_rot, z) != (rotated_coords[-1][0], rotated_coords[-1][1], z_values[-1]):
                timestamp = time_stamp / 1000.0  # Convert ms to seconds
                rotated_coords.append((x_rot, y_rot))
                timestamps.append(timestamp)
                z_values.append(z)

    for i in range(len(rotated_coords) - 1):
        x1, y1 = rotated_coords[i]
        x2, y2 = rotated_coords[i + 1]
        z1, z2 = z_values[i], z_values[i + 1]
        t1, t2 = timestamps[i], timestamps[i + 1]

        dt = t2 - t1 if t2 != t1 else 1e-6  # Avoid division by zero
        vx = (x2 - x1) / dt
        vy = (y2 - y1) / dt
        vz = (z2 - z1) / dt

        point = LocationGlobalRelative(
            vehicle.location.global_frame.lat + x1 * 1e-5,
            vehicle.location.global_frame.lon + y1 * 1e-5,
            z1
        )
        waypoints.append((t1, point, vx, vy, vz))

    # Handle the final waypoint with zero velocity
    x_final, y_final = rotated_coords[-1]
    z_final = z_values[-1]
    t_final = timestamps[-1]
    point_final = LocationGlobalRelative(
        vehicle.location.global_frame.lat + x_final * 1e-5,
        vehicle.location.global_frame.lon + y_final * 1e-5,
        z_final
    )
    waypoints.append((t_final, point_final, 0.0, 0.0, 0.0))
    return waypoints

def arm_and_takeoff(target_altitude):
    """Arms the drone and flies to the target altitude."""
    
    vehicle.mode = VehicleMode("GUIDED")
    while not vehicle.mode == "GUIDED":     
        time.sleep(0.1)

    vehicle.armed = True
    while not vehicle.armed:
        time.sleep(0.1)

    vehicle.simple_takeoff(0.5)

    # Wait until the vehicle reaches a safe height
    while True:
        if vehicle.location.global_relative_frame.alt >= 0.5 * 0.95:   
            break
        time.sleep(0.1)

def fly_to_waypoints(waypoints):
    def send_position_velocity(lat, lon, alt, vx, vy, vz):
        msg = vehicle.message_factory.set_position_target_global_int_encode(
            0,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111000000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            vx, vy, vz,
            0, 0, 0,
            0, 0
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

    
    mission_start_time = time.time()

    for i, (waypoint_time, point, vx, vy, vz) in enumerate(waypoints[1:], start=1):
        while True:
            current_time = time.time() - mission_start_time
            time_to_next = waypoint_time - current_time

            if time_to_next <= 0:
                
                send_position_velocity(point.lat, point.lon, point.alt, vx, vy, vz)
                break
            else:
                
                time.sleep(min(0.5, time_to_next))

def land():   
    vehicle.mode = VehicleMode("LAND")
    while vehicle.armed:       
        time.sleep(0.2)
  
if __name__ == "__main__":
    csv_filepath = "/home/pi/Drone_Swarm/Drone/Drone_1_complex.csv"
    waypoints = read_waypoints(csv_filepath)
    DroneController(vehicle)
    # for t, point, vx, vy, vz in waypoints:
    #     print(f"{t:.3f},{point.lat:.7f},{point.lon:.7f},{point.alt:.2f},{vx:.4f},{vy:.4f},{vz:.4f}")

    if waypoints:
        arm_and_takeoff(waypoints[1][1].alt) 
        while hello < 1700:
            time.sleep(0.1)
        fly_to_waypoints(waypoints)
        land()

vehicle.close()
