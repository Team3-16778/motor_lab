import sys
import serial
import threading
import time
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton, QSlider, QComboBox, QSpacerItem, QSizePolicy
from PyQt6.QtCore import Qt, QTimer

# === Serial Port ===
SERIAL_PORT = "/dev/cu.usbmodem2101"  # Windows: COM4, Linux/macOS: /dev/ttys001
BAUD_RATE = 9600

# Connect to the serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except:
    ser = None
    print("Warning: No connection, change a virtual serial port.")

# === Sensor & Motor ===
NUM_SENSORS = 3
NUM_MOTORS = 3
sensor_values = np.zeros((NUM_SENSORS, 100))  # Record sensor values, 100 data points
motor_states = [0] * NUM_MOTORS  # motor states
motor_modes = [0] * NUM_MOTORS  # 0: Reset & Stop, 1: GUI Control, 2: Sensor Auto Control
reading_sensors = False  # Check Botton of Sensor Data Reading
motor_ranges = [[0, 180], [0, 255], [0, 255]]
Default_motor_states = [90, 0, 0]

# === Matplotlib Canvas (PyQt6)===
class MatplotlibCanvas(FigureCanvas):
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        super().__init__(self.fig)
        self.ax.set_ylim(0, 1050)
        self.ax.set_xlim(0, 100)
        self.ax.set_title("Sensor Data")
        self.ax.set_xlabel("Time")
        self.ax.set_ylabel("Sensor Value")
        self.lines = [self.ax.plot([], [], label=f"Sensor {i+1}")[0] for i in range(NUM_SENSORS)]
        self.ax.legend(loc = "upper left")

# === GUI Interface ===
class ArduinoControlApp(QWidget):
    def __init__(self):
        
        super().__init__()
        self.setWindowTitle("Arduino Sensor & Motor Control GUI Interface - V0.2")
        self.setGeometry(100, 100, 900, 700)

        # Sensor labels & Matplotlib Canvas
        self.sensor_labels = [QLabel(f"Sensor {i+1}: --") for i in range(NUM_SENSORS)]
        self.canvas = MatplotlibCanvas()

        # Sensor toggle button
        self.sensor_toggle_button = QPushButton("Start Sensor Read")
        self.sensor_toggle_button.setCheckable(True)
        self.sensor_toggle_button.clicked.connect(self.toggle_sensor_reading)

        # Motor Control UI
        self.motor_sliders = []
        self.motor_states_labels = []  # motor state labels
        self.motor_modes_dropdowns = []
        self.motor_timers = [QTimer(self) for _ in range(NUM_MOTORS)]  # QTimer: limit serial write frequency


        for i in range(NUM_MOTORS):
            motor_layout = QHBoxLayout()

            # Motor name
            motor_label = QLabel(f"Motor {i+1}")

            # Control Mode Dropdown
            dropdown = QComboBox()
            dropdown.addItems(["Reset & Stop", "GUI Control", "Sensor Auto Control"])
            dropdown.currentIndexChanged.connect(lambda index, i=i: self.set_motor_mode(i, index))
            self.motor_modes_dropdowns.append(dropdown)

            # Motor States Control Slider
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setFixedWidth(400)  # slider width
            slider.setMinimum(motor_ranges[i][0])
            slider.setMaximum(motor_ranges[i][1])
            slider.setValue(Default_motor_states[i])
            slider.setEnabled(False)  # Default disabled
            slider.valueChanged.connect(lambda value, i=i: self.update_motor_states(i, value))  # valueChanged trigger
            self.motor_sliders.append(slider)

            # QLabel to display slider value
            states_label = QLabel(str(Default_motor_states[i]))  # Default value
            self.motor_states_labels.append(states_label)

            # QTimer to limit serial write frequency
            self.motor_timers[i].setSingleShot(True)
            self.motor_timers[i].timeout.connect(lambda i=i: self.set_motor_states(i, motor_states[i]))

            # Add widgets to layout
            motor_layout.addWidget(motor_label)
            motor_layout.addWidget(dropdown) 
            motor_layout.addWidget(slider) 
            motor_layout.addWidget(states_label)
            motor_layout.addStretch()

            setattr(self, f'motor_layout_{i}', motor_layout)
            setattr(self, f'motor_label_{i}', motor_label)

        # === Main Layout ===
        layout = QVBoxLayout()
        layout.addWidget(self.sensor_toggle_button)

        # Sensor labels layout
        sensor_layout = QHBoxLayout()  # Horizontal layout
        for label in self.sensor_labels:
            sensor_layout.addWidget(label)
        layout.addLayout(sensor_layout)
        
        # Matplotlib Canvas layout for sensor data
        layout.addWidget(self.canvas)

        # Motor layouts
        for i in range(NUM_MOTORS):
            layout.addLayout(getattr(self, f'motor_layout_{i}'))

        self.setLayout(layout)

        # Matplotlib Animation
        self.anim = animation.FuncAnimation(self.canvas.fig, self.update_plot, interval=100, blit=False)

    # Start/Stop Sensor Reading
    def toggle_sensor_reading(self):
        global reading_sensors
        if self.sensor_toggle_button.isChecked():
            self.sensor_toggle_button.setText("Stop Sensor Read")
            reading_sensors = True
            self.sensor_thread = threading.Thread(target=self.read_sensor_data)
            self.sensor_thread.start()
        else:
            self.sensor_toggle_button.setText("Start Sensor Read")
            reading_sensors = False

    # Read sensor data
    def read_sensor_data(self):
        global sensor_values
        while reading_sensors:
            try:
                if ser:
                    line = ser.readline().decode('utf-8').strip()
                    print(line)
                    if line.startswith("SENSOR:"):
                        values = list(map(int, line.split(":")[1].split(",")))
                        for i in range(NUM_SENSORS):
                            sensor_values[i] = np.roll(sensor_values[i], -1)
                            sensor_values[i][-1] = values[i]
                            self.sensor_labels[i].setText(f"Sensor {i+1}: {values[i]}")
                        
                        # If in auto control mode, update motor state by sensor data
                        for i in range(NUM_MOTORS):
                            if motor_modes[i] == 2:  # Auto Control
                                self.set_motor_states(i, values[i] // 4)  # Map sensor value to 0-255

            except:
                pass

    # Slider value changed
    def update_motor_states(self, motor_index, value):
        """ Update motor states to a specific value """
        if motor_modes[motor_index] == 1:  # GUI Control Mode ONLY
            self.motor_states_labels[motor_index].setText(str(value))  # update states label
            threading.Thread(target=self.send_motor_command, args=(motor_index, value), daemon=True).start()  # send motor command

    # Update motor states to a specific value
    def set_motor_states(self, motor_index, value):
        global motor_states
        if motor_modes[motor_index] == 1:  # GUI Control Mode ONLY
            motor_states[motor_index] = value
            self.motor_states_labels[motor_index].setText(str(value))  # update states label
            if ser:
                threading.Thread(target=self.send_motor_command, args=(motor_index, value), daemon=True).start()   
    # Send motor command
    def send_motor_command(self, motor_index, value):
        if ser:
            ser.write(f"MOTOR{motor_index+1}:{value}\n".encode('utf-8'))

    # Motor modes
    def set_motor_mode(self, motor_index, mode):
        global motor_modes
        motor_modes[motor_index] = mode
        if mode == 0:  # Reset & Stop
            self.motor_sliders[motor_index].setValue(Default_motor_states[motor_index])  # Set slider to Default
            self.motor_states_labels[motor_index].setText(str(Default_motor_states[motor_index]))  # Set state label to 0
            self.motor_sliders[motor_index].setEnabled(False)
            self.set_motor_states(motor_index, Default_motor_states[motor_index])
        elif mode == 1:  # GUI Control
            self.motor_sliders[motor_index].setEnabled(True)
        elif mode == 2:  # Auto Control
            self.motor_sliders[motor_index].setValue(Default_motor_states[motor_index]) 
            self.motor_states_labels[motor_index].setText(str(Default_motor_states[motor_index])) 
            self.motor_sliders[motor_index].setEnabled(False)
    
    # === Slider Update Rate Restriction ===
    def schedule_motor_update(self, motor_index, value):
        global motor_states
        motor_states[motor_index] = value
        self.motor_states_labels[motor_index].setText(str(value))
        self.motor_timers[motor_index].start(10)  # update motor state every 10ms

    # Update plot of sensor data
    def update_plot(self, frame):
        for i in range(NUM_SENSORS):
            self.canvas.lines[i].set_data(range(100), sensor_values[i])
        self.canvas.ax.relim()
        self.canvas.ax.autoscale_view()
        self.canvas.draw()

# Run GUI
app = QApplication(sys.argv)
window = ArduinoControlApp()
window.show()
sys.exit(app.exec())
