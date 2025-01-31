import sys
import serial
import threading
import time
from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QPushButton, QSlider
from PyQt6.QtCore import Qt

# === Serial Port Settings ===
SERIAL_PORT = '/dev/ttys032'  # Windows: COM3, Linux/macOS: /dev/ttys001
BAUD_RATE = 9600
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# === GUI Interface ===
class ArduinoControlApp(QWidget):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("Arduino Sensor & Motor Control")
        self.setGeometry(100, 100, 400, 300)

        # 传感器数据显示
        self.sensor_label = QLabel("Sensor Value: --", self)
        self.sensor_label.setAlignment(Qt.AlignmentFlag.AlignCenter)

        # 电机控制滑动条
        self.motor_slider = QSlider(Qt.Orientation.Horizontal, self)
        self.motor_slider.setMinimum(0)
        self.motor_slider.setMaximum(255)
        self.motor_slider.setValue(0)
        self.motor_slider.valueChanged.connect(self.set_motor_speed)

        # 停止电机按钮
        self.stop_button = QPushButton("Stop Motor", self)
        self.stop_button.clicked.connect(self.stop_motor)

        # 布局
        layout = QVBoxLayout()
        layout.addWidget(self.sensor_label)
        layout.addWidget(self.motor_slider)
        layout.addWidget(self.stop_button)
        self.setLayout(layout)

        # 启动传感器数据读取线程
        self.running = True
        self.sensor_thread = threading.Thread(target=self.read_sensor_data)
        self.sensor_thread.start()

    # 读取 Arduino 传感器数据
    def read_sensor_data(self):
        while self.running:
            try:
                line = ser.readline().decode('utf-8').strip()
                if line.startswith("SENSOR:"):
                    sensor_value = line.split(":")[1]
                    self.sensor_label.setText(f"Sensor Value: {sensor_value}")
            except:
                pass
            time.sleep(0.1)

    # 设置电机速度
    def set_motor_speed(self, value):
        command = f"MOTOR:{value}\n"
        ser.write(command.encode('utf-8'))

    # 停止电机
    def stop_motor(self):
        self.motor_slider.setValue(0)  # 复位滑动条
        self.set_motor_speed(0)

    # 关闭窗口时停止线程
    def closeEvent(self, event):
        self.running = False
        self.sensor_thread.join()
        ser.close()
        event.accept()

# === 启动 GUI ===
if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = ArduinoControlApp()
    window.show()
    sys.exit(app.exec())
