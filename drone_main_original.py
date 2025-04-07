import sys
import os
from PyQt5 import QtWidgets, uic
from PyQt5.QtWidgets import QLabel
from PyQt5.QtCore import pyqtSignal

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
        self.test_AI_code_lbl.mousePressEvent = lambda event: self.test_AI_code(event)
        self.up_lbl.mousePressEvent = lambda event: self.up(event)
        self.down_lbl.mousePressEvent = lambda event: self.down(event)
        self.left_lbl.mousePressEvent = lambda event: self.left(event)
        self.right_lbl.mousePressEvent = lambda event: self.right(event)

        self.show()

    def arm(self,event):
        print("Arming drone...")

    def takeoff(self,event):
        print("takeoff drone...")

    def land(self,event):
        print("land drone...")

    def disarm(self,event):
        print("disarm drone...")

    def test_AI_code(self,event):
        print("test_AI_code drone...")

    def up(self,event):
        print("up drone...")

    def down(self,event):
        print("up drone...")

    def left(self,event):
        print("left drone...")

    def right(self,event):
        print("right drone...")


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = MyWindow()
    window.setWindowTitle("Drone Control")
    sys.exit(app.exec_())
