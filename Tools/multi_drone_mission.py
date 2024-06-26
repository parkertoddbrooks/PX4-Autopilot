import time
from pymavlink import mavutil

# Connect to the drones (adjust the connection strings as needed)
drones = [
    mavutil.mavlink_connection('udpin:localhost:14540'),
    mavutil.mavlink_connection('udpin:localhost:14550'),
    mavutil.mavlink_connection('udpin:localhost:14560'),
    mavutil.mavlink_connection('udpin:localhost:14570'),
    mavutil.mavlink_connection('udpin:localhost:14580')
]

# Wait for heartbeats from all drones
for drone in drones:
    drone.wait_heartbeat()
    print(f"Heartbeat from drone (system {drone.target_system} component {drone.target_component})")

# Define waypoints for each drone (latitude, longitude, altitude)
waypoints = [
    [(47.3977419, 8.5455938, 10), (47.3977419, 8.5456938, 10)],  # Drone 1
    [(47.3978419, 8.5455938, 10), (47.3978419, 8.5456938, 10)],  # Drone 2
    [(47.3979419, 8.5455938, 10), (47.3979419, 8.5456938, 10)],  # Drone 3
    [(47.3980419, 8.5455938, 10), (47.3980419, 8.5456938, 10)],  # Drone 4
    [(47.3981419, 8.5455938, 10), (47.3981419, 8.5456938, 10)]   # Drone 5
]

def arm_and_takeoff(drone, target_altitude):
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
        1, 0, 0, 0, 0, 0, 0)

    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, target_altitude)

def goto_position(drone, lat, lon, alt):
    drone.mav.mission_item_send(
        drone.target_system, drone.target_component,
        0, 0,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
        mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
        0, 0, 0, 0, lat, lon, alt)

def land(drone):
    drone.mav.command_long_send(
        drone.target_system, drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
        0, 0, 0, 0, 0, 0, 0)

# Main mission loop
for i, drone in enumerate(drones):
    print(f"Starting mission for Drone {i+1}")
    arm_and_takeoff(drone, waypoints[i][0][2])
    
    for waypoint in waypoints[i]:
        goto_position(drone, *waypoint)
        time.sleep(10)  # Wait for the drone to reach the waypoint
        
        land(drone)
        print(f"Drone {i+1} landed. Waiting for 1 minute.")
        time.sleep(60)  # Wait for 1 minute
        
        if waypoint != waypoints[i][-1]:  # If it's not the last waypoint
            arm_and_takeoff(drone, waypoint[2])

    print(f"Mission completed for Drone {i+1}")

print("All missions completed")