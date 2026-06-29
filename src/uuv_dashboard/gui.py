import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Temperature, FluidPressure

import threading
import cv2
from PyQt6.QtWidgets import (QApplication, QWidget, QLabel,QVBoxLayout, QHBoxLayout,QProgressBar, QFrame)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QPixmap, QImage

class RobotSubscriber(Node): #Todo:Update to include other topics
     def __init__(self, external_temp_callback, thruster_callback, external_pressure_callback):
          super().__init__('battery_gui_subscriber')

          self.create_subscription(Temperature, 'baro/external/temperature', external_temp_callback, 10)
          self.create_subscription(Float32MultiArray, '/thruster/command', thruster_callback, 10)
          self.create_subscription(FluidPressure, 'baro/external/data', external_pressure_callback, 10)

class RobotGUI(QWidget):
     thrusters_updated = pyqtSignal(list)
     external_temp_updated = pyqtSignal(float)
     external_pressure_updated = pyqtSignal(float)
     
     
     def __init__(self):
          super().__init__()

# INITIALIZE VALUES
          self.TotalV= 0.0
          self.cells = [0.0] * 8

          self.battery_capacity= 0.0
          self.I = 0.0

          self.thrusters = [0.0] * 8

          self.internal_temp = 0.0
          self.external_temp = 0.0
          self.bms_temp_1 = 0.0
          self.bms_temp_2 = 0.0
          self.bms_temp_3 = 0.0

          self.x = 0.0
          self.y = 0.0

          self.external_pressure = 0.0
          self.internal_pressure = 0.0

          #connect signal to slots #todo: add in other on_..._update
          self.thrusters_updated.connect(self.on_thrusters_update)
          self.external_temp_updated.connect(self.on_external_temp_update)
          self.external_pressure_updated.connect(self.on_external_pressure_update)

          #start ros2 #todo: add in other self.ros_..._callback
          self.ros_node = RobotSubscriber(
               self.ros_external_temp_callback,
               self.ros_thruster_callback,
               self.ros_external_pressure_callback
          )

          self.ros_thread = threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True)
          self.ros_thread.start()

          self.init_ui() # build UI
     
     def ros_thruster_callback(self, msg: Float32MultiArray):
          self.thrusters_updated.emit(list(msg.data))

     def ros_external_temp_callback(self, msg: Temperature):
          self.external_temp_updated.emit(float(msg.temperature))

     def ros_external_pressure_callback(self, msg: FluidPressure):
          self.external_pressure_updated.emit(float(msg.fluid_pressure))
     
     #todo: add in other callback functions

     def on_thrusters_update(self, values: list):
          self.thrusters = values
          for i, label in enumerate(self.thruster_labels):
               label.setText(f"{self.thrusters[i]:.1f}%")

     def on_external_temp_update(self, value: float):
          self.external_temp = value
          self.external_temp_label.setText(f"External {self.external_temp:.1f} °C") 

     def on_external_pressure_update(self, value: float):
          self.external_pressure = value
          self.external_pressure_label.setText(f"External {self.external_pressure:.1f} Pa")

     #todo: add in other update functions

     def closeEvent(self, event):
          self.ros_node.destroy_node()
          rclpy.shutdown()
          event.accept()

#UI LAYOUT
     def init_ui(self):
          self.setWindowTitle("The Best Dashboard")
          self.setGeometry(100, 100, 1100, 550)

          main_layout = QHBoxLayout()

#LEFT COLUMN
          battery_col = QVBoxLayout()

          title = QLabel("Batteries")
          title.setStyleSheet("font-size: 18px; font-weight: bold;")
          battery_col.addWidget(title)

        #TotalV
          self.TotalV_label = QLabel(f"Total Voltage: {self.TotalV:.2f} V")
          battery_col.addWidget(self.TotalV_label)

        #cells
          self.cell_1_label = QLabel(f"Cell 1: {self.cells[0]} V")
          battery_col.addWidget(self.cell_1_label)

          self.cell_2_label = QLabel(f"Cell 2: {self.cells[1]} V")
          battery_col.addWidget(self.cell_2_label)

          self.cell_3_label = QLabel(f"Cell 3: {self.cells[2]} V")
          battery_col.addWidget(self.cell_3_label)

          self.cell_4_label = QLabel(f"Cell 4: {self.cells[3]} V")
          battery_col.addWidget(self.cell_4_label)

          self.cell_5_label = QLabel(f"Cell 5: {self.cells[4]} V")
          battery_col.addWidget(self.cell_5_label)

          self.cell_6_label = QLabel(f"Cell 6: {self.cells[5]} V")
          battery_col.addWidget(self.cell_6_label)

          self.cell_7_label = QLabel(f"Cell 7: {self.cells[6]} V")
          battery_col.addWidget(self.cell_7_label)

          self.cell_8_label = QLabel(f"Cell 8: {self.cells[7]} V")
          battery_col.addWidget(self.cell_8_label)
          
          battery_col.addSpacing(10)

          self.current_label = QLabel(f"Current: {self.I:.2f} A")
          self.battery_capacity_label = QLabel(f"Remaining Capacity: {self.battery_capacity} %")

          battery_col.addWidget(self.current_label)
          battery_col.addWidget(self.battery_capacity_label)

#MIDDLE COLUMN
          thruster_col = QVBoxLayout()

          thruster_title = QLabel("Thrusters")
          thruster_title.setStyleSheet("font-size: 18px; font-weight: bold;")
          thruster_col.addWidget(thruster_title)

          self.thruster_labels = []

          for i in range(8):
               row = QHBoxLayout()

               label = QLabel(f"Thruster {i+1}")
               label.setAlignment(Qt.AlignmentFlag.AlignVCenter)
               value = QLabel(f"{self.thrusters[i]:.1f}%")
               value.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)

               row.addWidget(label)
               row.addStretch()
               row.addWidget(value)

               thruster_col.addLayout(row)
               self.thruster_labels.append(value)

#RIGHT COLUMN
          right_col = QVBoxLayout()

        #temps
          temp_title = QLabel("Temperature")
          temp_title.setStyleSheet("font-size: 18px; font-weight: bold;")

          self.internal_temp_label = QLabel(f"Internal: {self.internal_temp:.1f} °C")
          self.external_temp_label = QLabel(f"External: {self.external_temp:.1f} °C")
          self.bms_temp_1_label = QLabel(f"Battery (probe 1): {self.bms_temp_1:.1f} °C")
          self.bms_temp_2_label = QLabel(f"Battery (probe 2): {self.bms_temp_2:.1f} °C")
          self.bms_temp_3_label = QLabel(f"Battery (probe 3): {self.bms_temp_3:.1f} °C")

          right_col.addWidget(temp_title)
          right_col.addWidget(self.internal_temp_label)
          right_col.addWidget(self.external_temp_label)
          right_col.addWidget(self.bms_temp_1_label)
          right_col.addWidget(self.bms_temp_2_label)
          right_col.addWidget(self.bms_temp_3_label)
          

          right_col.addSpacing(10)

          #pressures
          pressure_title = QLabel("Pressure")
          pressure_title.setStyleSheet("font-size: 18px; font-weight: bold;")

          self.internal_pressure_label = QLabel(f"Internal: {self.internal_pressure:.1f} Pa")
          self.external_pressure_label = QLabel(f"External: {self.external_pressure:.1f} Pa")

          right_col.addWidget(pressure_title)
          right_col.addWidget(self.internal_pressure_label)
          right_col.addWidget(self.external_pressure_label)
          
          right_col.addSpacing(10)


        #position and velocity
          pos_title = QLabel("Position")
          pos_title.setStyleSheet("font-size: 18px; font-weight: bold;")

          self.x_label = QLabel(f"X: {self.x:.2f}")
          self.y_label = QLabel(f"Y: {self.y:.2f}")


          right_col.addWidget(pos_title)
          right_col.addWidget(self.x_label)
          right_col.addWidget(self.y_label)
          

#CAMERA COLUMN
          camera_col = QVBoxLayout()

          camera_title = QLabel("Camera")
          camera_title.setStyleSheet("font-size: 18px; font-weight: bold;")
          camera_col.addWidget(camera_title)

        

# VERTICAL SEPARATORS
          line1 = self.make_vline()
          line2 = self.make_vline()
          line3 = self.make_vline()

          main_layout.addLayout(battery_col)
          main_layout.addWidget(line1)
          main_layout.addLayout(thruster_col)
          main_layout.addWidget(line2)
          main_layout.addLayout(right_col)
          main_layout.addWidget(line3)
          main_layout.addLayout(camera_col)


          main_layout.setStretch(0,2)
          main_layout.setStretch(2,2)
          main_layout.setStretch(4,2)
          main_layout.setStretch(6,2)

          self.setLayout(main_layout)

#HELPERS
     def make_bar(self, value):
          bar = QProgressBar()
          bar.setRange(0, 100)
          bar.setValue(int(value))
          bar.setFormat("%p%")
          return bar

     def make_vline(self):
          line = QFrame()
          line.setFrameShape(QFrame.Shape.VLine)
          line.setFrameShadow(QFrame.Shadow.Sunken)
          return line
            
     '''def update_data(self):

        self.batterypercent = max(0, self.batterypercent - random.randint(0,2))
        self.battery_bar.setValue(self.batterypercent)

        for i in range(8):
             self.cells[i] = max(0, self.cells[i] - random.randint(0,2))
             self.cell_bars[i].setValue(self.cells[i])
        
        self.V = 28 + random.uniform(-1,1)
        self.I = 5 + random.uniform(-1,1)
        self.voltage_label.setText(f"Voltage: {self.V:.2f} V")
        self.current_label.setText(f"Current: {self.I: .2f} A")


        self.internaltemperature = random.uniform(20, 30)
        self.batterytemperature = random.uniform(20, 30)'''

if __name__ == "__main__":
     rclpy.init()
     app = QApplication(sys.argv)
     window = RobotGUI()
     window.show()
     sys.exit(app.exec())
