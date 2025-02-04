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
BAUD_RATE = 115200

# Connect to the serial port
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
except:
    ser = None
    print("====== Warning: Wrong Serial Port, NO CONNECTION !!! ======")

# === Sensor & Motor ===
NUM_SENSORS = 3
sensor_names = ["Potentiometer", "Photoresistor", "Flux"]
sensor_values = np.zeros((NUM_SENSORS, 100))  # Record sensor values, 100 data points
reading_sensors = False  # Check Botton of Sensor Data Reading
sensor_data_ranges = [[0, 10000], [0, 100], [-120, 120]]

NUM_MOTORS = 3
motor_names = ["Servo", "Stepper", "DC with Encoder"]
motor_states = [0] * NUM_MOTORS  # motor states
motor_modes = [0] * NUM_MOTORS  # 0: Reset & Stop, 1: GUI Control, 2: Sensor Auto Control
motor_ranges_display = [[0, 180], [-180, 180], [0, 230]]
motor_ranges = [[0, 180], [-2048, 2048], [0, 230]]
default_motor_states = [90, 0, 0]

## TODO: Using PyQtGraph to plot sensor data (Faster than Matplotlib)
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
        self.setWindowTitle("Arduino Sensor & Motor Control GUI Interface - V0.3")
        self.setGeometry(100, 100, 900, 700)

        # Sensor labels & Matplotlib Canvas
        self.sensor_labels = [QLabel(f"Sensor {i+1} ({sensor_names[i]}): --") for i in range(NUM_SENSORS)]
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
            motor_label = QLabel(f"Motor {i+1} ({motor_names[i]}):")

            # Control Mode Dropdown
            dropdown = QComboBox()
            dropdown.addItems(["Reset & Stop", "GUI Control", "Auto Control"])
            dropdown.currentIndexChanged.connect(lambda index, i=i: self.set_motor_mode(i, index))
            self.motor_modes_dropdowns.append(dropdown)

            # Motor States Control Slider
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setFixedWidth(400)  # slider width
            slider.setMinimum(motor_ranges_display[i][0])
            slider.setMaximum(motor_ranges_display[i][1])
            slider.setValue(default_motor_states[i])
            slider.setEnabled(False)  # Default disabled
            slider.valueChanged.connect(lambda value, i=i: self.update_slider_value(i, value))  # valueChanged trigger
            self.motor_sliders.append(slider)

            # QLabel to display slider value
            states_label = QLabel(str(default_motor_states[i]))  # Default value
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
                        values = list(map(float, line.split(":")[1].strip().split(",")))
                        for i in range(NUM_SENSORS):
                            sensor_values[i] = np.roll(sensor_values[i], -1)
                            sensor_values[i][-1] = values[i]
                            self.sensor_labels[i].setText(f"Sensor {i+1} ({sensor_names[i]}): {values[i]}")
                        
                        # If in auto control mode, update motor state by sensor data
                        for i in range(NUM_MOTORS):
                            if motor_modes[i] == 2:  # Auto control by sensor
                                # Map sensor value to motor display range
                                value = np.interp(values[i], sensor_data_ranges[i], motor_ranges_display[i])
                                self.set_motor_states(i, value)
            except:
                pass

    # Update slider value
    def update_slider_value(self, motor_index, value):
        """ Update motor states to a specific value """
        if motor_modes[motor_index] == 1:  # GUI Control Mode ONLY
            self.motor_states_labels[motor_index].setText(str(value))  # update states label
            # map slider value to motor range
            value_mapped = np.interp(value, motor_ranges_display[motor_index], motor_ranges[motor_index])
            #print(value,value_mapped)
            threading.Thread(target=self.send_motor_command, args=(motor_index, value_mapped), daemon=True).start()  # send motor command

    # Update motor states to a specific value
    def set_motor_states(self, motor_index, value):
        global motor_states
        motor_states[motor_index] = value
        self.motor_states_labels[motor_index].setText(str(value))  # update states label
        if ser:
            # map slider value to motor range
            value_mapped = np.interp(value, motor_ranges_display[motor_index], motor_ranges[motor_index])
            threading.Thread(target=self.send_motor_command, args=(motor_index, value_mapped), daemon=True).start() 
  
    # Send motor command
    def send_motor_command(self, motor_index, value):
        if ser:
            ser.write(f"MOTOR{motor_index+1}:{value}\n".encode('utf-8'))

    # Motor modes
    def set_motor_mode(self, motor_index, mode):
        global motor_modes
        motor_modes[motor_index] = mode
        if mode == 0:  # Reset & Stop
            self.motor_sliders[motor_index].setValue(default_motor_states[motor_index])  # Set slider to Default
            self.motor_states_labels[motor_index].setText(str(default_motor_states[motor_index]))  # Set state label to 0
            self.motor_sliders[motor_index].setEnabled(False)
            self.set_motor_states(motor_index, default_motor_states[motor_index])
        elif mode == 1:  # GUI Control
            self.motor_sliders[motor_index].setEnabled(True)
        elif mode == 2:  # Auto Control
            self.motor_sliders[motor_index].setValue(default_motor_states[motor_index]) 
            self.motor_states_labels[motor_index].setText(str(default_motor_states[motor_index])) 
            self.motor_sliders[motor_index].setEnabled(False)
    
    # === Slider Update Rate Restriction ===
    def schedule_motor_update(self, motor_index, value):
        global motor_states
        motor_states[motor_index] = value
        self.motor_states_labels[motor_index].setText(str(value))
        self.motor_timers[motor_index].start(10)  # update motor state every 10ms

    # Update plot of sensor data
    def update_plot(self, frame):
        """ Update the plot with three separate subplots for each sensor """
        
        # Clear previous figure
        self.canvas.fig.clear()

        # Create 3 subplots (one for each sensor)
        ax1, ax2, ax3 = self.canvas.fig.subplots(3, 1, sharex=True)  # 3 rows, 1 column, shared x-axis

        # Plot Sensor 1
        ax1.plot(range(100), sensor_values[0], 'b', label="Sensor 1 ({})".format(sensor_names[0]))
        ax1.set_ylabel("Sensor1 (Ohm)")
        ax1.set_title("Sensor Data Over Time")
        ax1.legend(loc="upper left")
        ax1.set_ylim(sensor_data_ranges[0][0], sensor_data_ranges[0][1])
        
        # Plot Sensor 2
        ax2.plot(range(100), sensor_values[1], 'r', label="Sensor 2 ({})".format(sensor_names[1]))
        ax2.set_ylabel("Sensor2 (%)")
        ax2.legend(loc="upper left")
        ax2.set_ylim(sensor_data_ranges[1][0], sensor_data_ranges[1][1])

        # Plot Sensor 3
        ax3.plot(range(100), sensor_values[2], 'g', label="Sensor 3 ({})".format(sensor_names[2]))
        ax3.set_ylabel("Sensor3 (degrees)")
        ax3.set_xlabel("Time (Samples)")  # X-axis only on the last subplot
        ax3.legend(loc="upper left")
        ax3.set_ylim(sensor_data_ranges[2][0], sensor_data_ranges[2][1]) 

        # Adjust layout to prevent overlapping
        self.canvas.fig.tight_layout()

        # Redraw the canvas
        self.canvas.draw()


# Run GUI
app = QApplication(sys.argv)
window = ArduinoControlApp()
window.show()
sys.exit(app.exec())
