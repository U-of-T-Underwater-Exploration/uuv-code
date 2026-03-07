import sys
from PyQt6.QtWidgets import (
QApplication, QWidget, QLabel,
QVBoxLayout, QHBoxLayout,
QProgressBar, QFrame
)
from PyQt6.QtCore import Qt, QTimer
import random


class RobotGUI(QWidget):
    def __init__(self):
        super().__init__()

# ================= INITIALIZE VALUES =================
        self.batterypercent = 100
        self.cells = [100] * 8

        self.V = 30.0
        self.I = 5.0

        self.thrusters = [0.0] * 8

        self.internal_temp = 25.0
        self.battery_temp = 28.0

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.vx = 0.0
        self.vy = 0.0
        self.vz = 0.0

        #TIMER
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_data)
        self.timer.start(1000) #update every second

# Build UI
        self.init_ui()

# ================= UI LAYOUT =================
    def init_ui(self):
        self.setWindowTitle("Robot Dashboard")
        self.setGeometry(100, 100, 1100, 550)

        main_layout = QHBoxLayout()

# ================= LEFT COLUMN (BATTERIES) =================
        battery_col = QVBoxLayout()

        title = QLabel("Batteries")
        title.setStyleSheet("font-size: 18px; font-weight: bold;")
        battery_col.addWidget(title)

# Main battery
        battery_col.addWidget(QLabel("Battery"))
        self.battery_bar = self.make_bar(self.batterypercent)
        battery_col.addWidget(self.battery_bar)

# Cell bars
        self.cell_bars = []
        for i in range(8):
                battery_col.addWidget(QLabel(f"Cell {i+1}"))
                bar = self.make_bar(self.cells[i])
                self.cell_bars.append(bar)
                battery_col.addWidget(bar)

        battery_col.addSpacing(10)

        self.voltage_label = QLabel(f"Voltage: {self.V:.2f} V")
        self.current_label = QLabel(f"Current: {self.I:.2f} A")

        battery_col.addWidget(self.voltage_label)
        battery_col.addWidget(self.current_label)

# ================= MIDDLE COLUMN (THRUSTERS) =================
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

# ================= RIGHT COLUMN =================
        right_col = QVBoxLayout()

        temp_title = QLabel("Temperature")
        temp_title.setStyleSheet("font-size: 18px; font-weight: bold;")

        self.internal_temp_label = QLabel(f"Internal: {self.internal_temp:.1f} °C")
        self.battery_temp_label = QLabel(f"Battery: {self.battery_temp:.1f} °C")

        right_col.addWidget(temp_title)
        right_col.addWidget(self.internal_temp_label)
        right_col.addWidget(self.battery_temp_label)

        right_col.addSpacing(20)

        pos_title = QLabel("Position")
        pos_title.setStyleSheet("font-size: 18px; font-weight: bold;")

        self.x_label = QLabel(f"X: {self.x:.2f}")
        self.y_label = QLabel(f"Y: {self.y:.2f}")
        self.z_label = QLabel(f"Z: {self.z:.2f}")

        self.vx_label = QLabel(f"Vx: {self.vx:.2f}")
        self.vy_label = QLabel(f"Vy: {self.vy:.2f}")
        self.vz_label = QLabel(f"Vz: {self.vz:.2f}")

        right_col.addWidget(pos_title)
        right_col.addWidget(self.x_label)
        right_col.addWidget(self.y_label)
        right_col.addWidget(self.z_label)
        right_col.addSpacing(10)
        right_col.addWidget(self.vx_label)
        right_col.addWidget(self.vy_label)
        right_col.addWidget(self.vz_label)

# ================= VERTICAL SEPARATORS =================
        line1 = self.make_vline()
        line2 = self.make_vline()

        main_layout.addLayout(battery_col)
        main_layout.addWidget(line1)
        main_layout.addLayout(thruster_col)
        main_layout.addWidget(line2)
        main_layout.addLayout(right_col)

        main_layout.setStretch(0,3)
        main_layout.setStretch(2,2)
        main_layout.setStretch(4,2)

        self.setLayout(main_layout)

# ================= HELPERS =================
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
    
    def update_data(self):

        self.batterypercent = max(0, self.batterypercent - random.randint(0,2))
        self.battery_bar.setValue(self.batterypercent)

        for i in range(8):
             self.cells[i] = max(0, self.cells[i] - random.randint(0,2))
             self.cell_bars[i].setValue(self.cells[i])
        
        self.v = 28 + random.uniform(-1,1)
        self.I = 5 + random.uniform(-1,1)
        self.voltage_label.setText(f"Voltage: {self.V:.2f} V")
        self.current_label.setText(f"Current: {self.I: .2f} A")


        self.internaltemperature = random.uniform(20, 30)
        self.batterytemperature = random.uniform(20, 30)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = RobotGUI()
    window.show()
    sys.exit(app.exec())