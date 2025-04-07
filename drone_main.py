import sys
import os
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import pyqtSignal
import subprocess

# Import functions from the second file
from command_for_drone import connect_to_drone, arm_drone, set_mode, takeoff, get_current_location, move_to, land, disarm_drone  

# Custom Clickable QLabel
class ClickableLabel(QLabel):
    clicked = pyqtSignal()  # Define a signal for clicks

    def mousePressEvent(self, event):
        self.clicked.emit()  # Emit the signal when clicked

class MyWindow(QtWidgets.QMainWindow):
    def __init__(self):
        super(MyWindow, self).__init__()

        # Load UI
        try:
            uic.loadUi('drone.ui', self)
        except Exception as e:
            print(f"Error loading UI file: {e}")
            sys.exit(1)
       

        self.arm_lbl.mousePressEvent = lambda event: self.arm(event)
        self.takeoff_lbl.mousePressEvent = lambda event: self.takeoff(event)
        self.land_lbl.mousePressEvent = lambda event: self.land(event)
        self.disarm_lbl.mousePressEvent = lambda event: self.disarm(event)
        self.up_lbl.mousePressEvent = lambda event: self.move_forward(event)
        self.down_lbl.mousePressEvent = lambda event: self.move_backward(event)
        self.left_lbl.mousePressEvent = lambda event: self.move_left(event)
        self.right_lbl.mousePressEvent = lambda event: self.move_right(event)
        self.test_AI_code_lbl.mousePressEvent = lambda event: self.test_AI_code(event)

        self.show()

    def arm(self, event):
        print("connecting with drone...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        if not mavlink_connection:
            print("[info]    Mavlink not connected")
             #return

        if not set_mode(mavlink_connection, "GUIDED"):
            print("[info]    GUIDED")
            #return
        print("arming drone...")
        if not arm_drone(mavlink_connection):
            print("[info]    arm")
            #return


        #print("connecting with drone...")
        ## Establish MAVLink connection
        #mavlink_connection = connect_to_drone()
        #print("arming drone...")
        #if self.mavlink_connection:
        #    arm_drone(mavlink_connection)
        #else:
        #    print("MAVLink connection not established.")

    def takeoff(self, event):
        print("Takeoff drone...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        if not takeoff(mavlink_connection, 3):
            print("[info]    takeoff")
            time.sleep(15)
            #return
        # Get current GPS location
        lat, lon, alt = get_current_location(mavlink_connection)
        if lat is None or lon is None:
            print("❌ No valid GPS location. Emergency landing!")
            land(mavlink_connection)
            #return
            print("NO GPS")

    def land(self, event):
        print("Landing drone...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        land(mavlink_connection)
        time.sleep(5)

    def disarm(self, event):
        print("Disarm drone...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        disarm_drone(mavlink_connection)



    def move_forward(self, event):
        print("move move_forward...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        lat, lon, alt = get_current_location(mavlink_connection)
        # Define waypoints 
        waypoints = [
            (lat, lon + 0.00001, 3),  # Move north forward
            ]

        # Move to each waypoint
        for wp in waypoints:
            move_to(mavlink_connection, *wp)
        #time.sleep(5)
    def move_backward(self, event):
        print("move move_backward...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        lat, lon, alt = get_current_location(mavlink_connection)
        # Define waypoints 
        waypoints = [
            (lat, lon - 0.00001, 3),   # Move south backward
            ]

        # Move to each waypoint
        for wp in waypoints:
            move_to(mavlink_connection, *wp)
        #time.sleep(5)
    def move_left(self, event):
        print("move Right...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        lat, lon, alt = get_current_location(mavlink_connection)
        # Define waypoints 
        waypoints = [
            (lat - 0.00001, lon, 3),  # Move west left
            ]

        # Move to each waypoint
        for wp in waypoints:
            move_to(mavlink_connection, *wp)
        #time.sleep(5)
    def move_right(self, event):
        print("move Right...")
        mavlink_connection = connect_to_drone()        #will be adjusted inside __init__
        lat, lon, alt = get_current_location(mavlink_connection)
        # Define waypoints 
        waypoints = [
            (lat + 0.00001, lon, 3),  # Move east right
            ]

        # Move to each waypoint
        for wp in waypoints:
            move_to(mavlink_connection, *wp)
        #time.sleep(5)
    def test_AI_code(self, event):
        import subprocess

        try:
            result = subprocess.run(
                ["bash", "commands_for_tracking.sh", "--verbose"],  # Run with bash
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                universal_newlines=True,
                check=True,
            )
            print("✅ Script output:\n", result.stdout)
        except subprocess.CalledProcessError as e:
            print("❌ Script failed!\n", e.stderr)

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.setWindowTitle("Drone Control")
    sys.exit(app.exec_())

