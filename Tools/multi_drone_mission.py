import time
import logging
from pymavlink import mavutil
from typing import List, Tuple
import threading

# Set up logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)

# Constants
HEARTBEAT_TIMEOUT = 5  # seconds
BATTERY_LOW_THRESHOLD = 20  # percent
GPS_FIX_TYPE_3D = 3
MISSION_TIMEOUT = 300  # seconds

class Drone:
    def __init__(self, connection_string: str):
        self.connection = mavutil.mavlink_connection(connection_string)
        self.target_system = None
        self.target_component = None
        self.last_heartbeat = 0
        self.mission_start_time = 0

    def wait_for_connection(self):
        logger.info(f"Waiting for heartbeat from {self.connection.port}")
        self.connection.wait_heartbeat()
        self.target_system = self.connection.target_system
        self.target_component = self.connection.target_component
        logger.info(f"Heartbeat from drone (system {self.target_system} component {self.target_component})")

    def check_connection(self):
        if time.time() - self.last_heartbeat > HEARTBEAT_TIMEOUT:
            raise ConnectionError(f"Lost connection to drone {self.target_system}")

    def arm_and_takeoff(self, target_altitude: float):
        self.connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        
        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError("Arming failed")

        self.connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, target_altitude)

        msg = self.connection.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if not msg or msg.result != mavutil.mavlink.MAV_RESULT_ACCEPTED:
            raise RuntimeError("Takeoff failed")

    def goto_position(self, lat: float, lon: float, alt: float):
        self.connection.mav.mission_item_send(
            self.target_system, self.target_component,
            0, 0,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0, 0, 0, 0, lat, lon, alt)

    def land(self):
        self.connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0)

    def return_to_launch(self):
        self.connection.mav.command_long_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH, 0,
            0, 0, 0, 0, 0, 0, 0)

    def check_battery(self):
        self.connection.mav.request_data_stream_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, 1, 1)
        msg = self.connection.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
        if msg and msg.battery_remaining < BATTERY_LOW_THRESHOLD:
            raise RuntimeError(f"Low battery: {msg.battery_remaining}%")

    def check_gps(self):
        self.connection.mav.request_data_stream_send(
            self.target_system, self.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION, 1, 1)
        msg = self.connection.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg and msg.fix_type < GPS_FIX_TYPE_3D:
            raise RuntimeError(f"Poor GPS signal: fix type {msg.fix_type}")

def monitor_drone(drone: Drone):
    while True:
        try:
            drone.check_connection()
            drone.check_battery()
            drone.check_gps()
            if drone.mission_start_time and time.time() - drone.mission_start_time > MISSION_TIMEOUT:
                raise TimeoutError("Mission timeout")
        except Exception as e:
            logger.error(f"Error in drone {drone.target_system}: {str(e)}")
            try:
                drone.return_to_launch()
            except:
                logger.error(f"Failed to initiate RTL for drone {drone.target_system}")
        time.sleep(1)

def run_mission(drone: Drone, waypoints: List[Tuple[float, float, float]]):
    try:
        drone.wait_for_connection()
        drone.mission_start_time = time.time()
        
        drone.arm_and_takeoff(waypoints[0][2])
        logger.info(f"Drone {drone.target_system} armed and taken off")

        for waypoint in waypoints:
            drone.goto_position(*waypoint)
            logger.info(f"Drone {drone.target_system} moving to waypoint {waypoint}")
            time.sleep(10)  # Wait for the drone to reach the waypoint

        drone.land()
        logger.info(f"Drone {drone.target_system} landing")
        time.sleep(60)  # Wait for 1 minute after landing

    except Exception as e:
        logger.error(f"Error in mission for drone {drone.target_system}: {str(e)}")
        try:
            drone.return_to_launch()
            logger.info(f"Initiated RTL for drone {drone.target_system}")
        except:
            logger.error(f"Failed to initiate RTL for drone {drone.target_system}")

# Main script
if __name__ == "__main__":
    # Connect to the drones (adjust the connection strings as needed)
    drones = [
        Drone('udpin:localhost:14540'),
        Drone('udpin:localhost:14550'),
        Drone('udpin:localhost:14560'),
        Drone('udpin:localhost:14570'),
        Drone('udpin:localhost:14580')
    ]

    # Define waypoints for each drone (latitude, longitude, altitude)
    waypoints = [
        [(47.3977419, 8.5455938, 10), (47.3977419, 8.5456938, 10)],  # Drone 1
        [(47.3978419, 8.5455938, 10), (47.3978419, 8.5456938, 10)],  # Drone 2
        [(47.3979419, 8.5455938, 10), (47.3979419, 8.5456938, 10)],  # Drone 3
        [(47.3980419, 8.5455938, 10), (47.3980419, 8.5456938, 10)],  # Drone 4
        [(47.3981419, 8.5455938, 10), (47.3981419, 8.5456938, 10)]   # Drone 5
    ]

    # Start monitoring threads for each drone
    monitor_threads = [threading.Thread(target=monitor_drone, args=(drone,), daemon=True) for drone in drones]
    for thread in monitor_threads:
        thread.start()

    # Run missions for each drone
    mission_threads = [threading.Thread(target=run_mission, args=(drone, waypoints[i])) for i, drone in enumerate(drones)]
    for thread in mission_threads:
        thread.start()

    # Wait for all mission threads to complete
    for thread in mission_threads:
        thread.join()

    logger.info("All missions completed")