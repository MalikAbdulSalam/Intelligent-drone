import time
from pymavlink import mavutil
import subprocess
import cv2
from ultralytics import YOLO
from command_for_drone import connect_to_drone, arm_drone, set_mode, takeoff, get_current_location, move_to, land, disarm_drone

#mavlink_connection = connect_to_drone()

# Get current GPS location
#lat, lon, alt = get_current_location(mavlink_connection)
#if lat is None or lon is None:
#    print("❌ No valid GPS location. Emergency landing!")
#    land(mavlink_connection)
#    #return
#    print("NO GPS")


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
                            (lat + 0.00001, lon, 3),  # Move east right
                            (lat - 0.00001, lon, 3)  # Move back to original possition (this will be deleted after installation of jetson on drone)
        
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)

                        elif bbox_center[0] > frame_center[0] + 90:
                            direction = "Move Left"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat - 0.00001, lon, 3),  # Move west left
                            (lat + 0.00001, lon, 3)  # Move back to original possition (this will be deleted after installation of jetson on drone)
                            
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)
                        if bbox_center[1] < frame_center[1] - 90:
                            direction += " Move Down"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat, lon - 0.00001, 3),   # Move south backward
                            (lat, lon + 0.00001, 3)   # Move back to original possition (this will be deleted after installation of jetson on drone)
                            ]

                            # Move to each waypoint
                            for wp in waypoints:
                                move_to(mavlink_connection, *wp)
                                #time.sleep(5)
                        elif bbox_center[1] > frame_center[1] + 90:
                            direction += " Move Up"
                            # Define waypoints 
                            waypoints = [
                            
                            (lat, lon + 0.00001, 3),  # Move north forward
                            (lat, lon - 0.00001, 3)  # Move back to original possition (this will be deleted after installation of jetson on drone)
                            
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

