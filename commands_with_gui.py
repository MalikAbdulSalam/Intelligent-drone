import time
from pymavlink import mavutil
import subprocess
import cv2
from ultralytics import YOLO

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
            print("❌ No GPS data received.")
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


mavlink_connection = connect_to_drone()
if not mavlink_connection:
    print("[info]    Mavlink not connected")
    #return

if not set_mode(mavlink_connection, "GUIDED"):
    print("[info]    GUIDED")
    #return

if not arm_drone(mavlink_connection):
    print("[info]    arm")
    #return

if not takeoff(mavlink_connection, 5):
    print("[info]    takeoff")
    #return
    
time.sleep(15)	
	
# Get current GPS location
lat, lon, alt = get_current_location(mavlink_connection)
if lat is None or lon is None:
    print("❌ No valid GPS location. Emergency landing!")
    land(mavlink_connection)
    #return
    print("NO GPS")
#############
# Load the YOLOv8 model
model = YOLO("yolov8n.pt")

# Open the video source (camera or video file)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open camera or video file.")
    exit()

prev_time = 0
fps = 0

selected_bbox = None
selected_id = None
selected_class = None
selected_confidence = None
tracking_initialized = False

# Mouse callback function
def select_object(event, x, y, flags, param):
    global selected_bbox, selected_id, selected_class, selected_confidence, tracking_initialized
    if event == cv2.EVENT_LBUTTONDOWN:
        for result in results:
            bbox = result.boxes.xyxy[0].cpu().numpy()
            if bbox[0] < x < bbox[2] and bbox[1] < y < bbox[3]:
                selected_bbox = bbox
                print(f"Selected object at: {bbox}")
                selected_id = result.boxes.id[0].item()
                selected_class = result.names[int(result.boxes.cls[0].item())]
                selected_confidence = result.boxes.conf[0].item()
                tracking_initialized = True
                break

cv2.namedWindow("YOLOv8 Tracking")
cv2.setMouseCallback("YOLOv8 Tracking", select_object)

while cap.isOpened():
    success, frame = cap.read()
    if success:
        current_time = time.time()
        fps = 1 / (current_time - prev_time)
        prev_time = current_time

        results = model.track(frame, persist=True)
        annotated_frame = frame.copy()
        frame_height, frame_width = frame.shape[:2]
        frame_center = (frame_width // 2, frame_height // 2)

        if tracking_initialized and selected_id is not None:
            for result in results:
                for i, bbox in enumerate(result.boxes.xyxy):
                    if result.boxes.id[i].item() == selected_id:
                        selected_bbox = bbox
                        cv2.rectangle(annotated_frame, 
                                      (int(selected_bbox[0]), int(selected_bbox[1])), 
                                      (int(selected_bbox[2]), int(selected_bbox[3])), 
                                      (0, 255, 0), 2)
                        label = f"ID: {selected_id}, Class: {selected_class}, Confidence: {selected_confidence:.2f}"
                        cv2.putText(annotated_frame, label, 
                                    (int(selected_bbox[0]), int(selected_bbox[1])-10), 
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        bbox_center = ((int(selected_bbox[0]) + int(selected_bbox[2])) // 2,
                                       (int(selected_bbox[1]) + int(selected_bbox[3])) // 2)
                        cv2.circle(annotated_frame, bbox_center, 50, (0, 0, 255), -1)
                        cv2.circle(annotated_frame, frame_center, 50, (255, 0, 0), -1)

                        direction = ""
                        if bbox_center[0] < frame_center[0] - 90:
                            direction = "Move Right"
                            # Define waypoints 
                            waypoints = [
                            (lat + 0.00003, lon, 5),  # Move east right
                            (lat - 0.00003, lon, 5)  # Move back to original possition (this will be deleted after installation of jetson on drone)
        
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)

                        elif bbox_center[0] > frame_center[0] + 90:
                            direction = "Move Left"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat - 0.00003, lon, 5),  # Move west left
                            (lat + 0.00003, lon, 5)  # Move back to original possition (this will be deleted after installation of jetson on drone)
                            
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)
                        if bbox_center[1] < frame_center[1] - 90:
                            direction += " Move Down"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat, lon - 0.00003, 5),   # Move south backward
                            (lat, lon + 0.00003, 5)   # Move back to original possition (this will be deleted after installation of jetson on drone)
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)
                        elif bbox_center[1] > frame_center[1] + 90:
                            direction += " Move Up"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat, lon + 0.00003, 5),  # Move north forward
                            (lat, lon - 0.00003, 5)  # Move back to original possition (this will be deleted after installation of jetson on drone)
                            
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)

                        if direction:
                            print(direction)
                            cv2.putText(annotated_frame, direction, (50, 50), 
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                        break
        cv2.imshow("YOLOv8 Tracking", annotated_frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

##########


# Define waypoints (Adjust these values based on your GPS location)
#waypoints = [
#    (lat + 0.00003, lon, 5),  # Move east right
#    (lat, lon + 0.00003, 5),  # Move north forward
#    (lat - 0.00003, lon, 5),  # Move west left
#    (lat, lon - 0.00003, 5)   # Move south back
#]

# Move to each waypoint
#for wp in waypoints:
#    move_to(mavlink_connection, *wp)
#    time.sleep(5)

# Land the drone
land(mavlink_connection)
time.sleep(5)
disarm_drone(mavlink_connection)
