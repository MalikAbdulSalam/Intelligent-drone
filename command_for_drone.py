import time
from pymavlink import mavutil
import subprocess
import cv2


# Connect to the drone
def connect_to_drone(connection_string="/dev/ttyUSB0", baudrate=57600):
    print(f"🔌 Connecting to {connection_string}...")
    try:
        mavlink_connection = mavutil.mavlink_connection(connection_string, baud=baudrate)
        mavlink_connection.wait_heartbeat()
        print(f"✅ Connected to Target System: {mavlink_connection.target_system}, Target Component: {mavlink_connection.target_component}")
        return mavlink_connection
    except Exception as e:
        print(f"❌ Connection error: {e}")
        return None

# Set flight mode
def set_mode(mavlink_connection, mode="GUIDED"):
    print(f"🔄 Switching to {mode} mode...")
    try:
        mode_id = mavlink_connection.mode_mapping()[mode]
        mavlink_connection.mav.set_mode_send(
            mavlink_connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        print(f"✅ Current Mode: {mode}")
        return True
    except KeyError:
        print(f"❌ Mode {mode} not supported")
        return False
    except Exception as e:
        print(f"❌ Error setting mode: {e}")
        return False

# Arm the drone
def arm_drone(mavlink_connection):
    print("🔄 Arming drone...")
    try:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system, mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0
        )
        print("✅ Drone armed.")
        return True
    except Exception as e:
        print(f"❌ Error arming drone: {e}")
        return False

# Takeoff function
def takeoff(mavlink_connection, altitude=5):
    print(f"📼 Taking off to {altitude} meters...")
    try:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system, mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
            0, 0, 0, 0, 0, 0, altitude
        )
        time.sleep(5)
        print("✅ Takeoff command sent.")
        return True
    except Exception as e:
        print(f"❌ Takeoff failed: {e}")
        return False


# Get current GPS location
def get_current_location(mavlink_connection):
    print("📡 Waiting for GPS data...")
    try:
        msg = mavlink_connection.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f"📍 Current Position - Lat: {lat}, Lon: {lon}, Alt: {alt}")
            return lat, lon, alt
        else:
            print("❌ No GPS data receicommand_for_droneved.")
            return None, None, None
    except Exception as e:
        print(f"❌ Error getting GPS data: {e}")
        return None, None, None

# Move to a specific waypoint
def move_to(mavlink_connection, lat, lon, alt):
    print(f"🛰️ Moving to: Lat {lat}, Lon {lon}, Alt {alt} meters...")
    try:
        mavlink_connection.mav.set_position_target_global_int_send(
            0, mavlink_connection.target_system, mavlink_connection.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  
            int(0b110111111000),  # Type mask (ignore velocity)
            int(lat * 1e7), int(lon * 1e7), alt,  # Target position
            0, 0, 0,  # Velocity
            0, 0, 0,  # Acceleration
            0, 0  # Yaw
        )
        print("✅ Move command sent.")
    except Exception as e:
        print(f"❌ Error moving to waypoint: {e}")

# Land the drone
def land(mavlink_connection):
    print("🛬 Landing...")
    try:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system, mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("✅ Land command sent.")
    except Exception as e:
        print(f"❌ Error landing: {e}")

# Disarm the drone
def disarm_drone(mavlink_connection):
    print("🔄 Disarming drone...")
    try:
        mavlink_connection.mav.command_long_send(
            mavlink_connection.target_system, mavlink_connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0
        )
        print("✅ Drone disarmed.")
    except Exception as e:
        print(f"❌ Error disarming drone: {e}")






