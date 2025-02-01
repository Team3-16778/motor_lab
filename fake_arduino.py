import time
import random
import serial

# Create virtual Arduino（Windows: COMx, macOS/Linux: /dev/ttySx or /dev/pts/x）
SERIAL_PORT = '/dev/ttys031'  # Windows: COM4, Linux/macOS: /dev/ttys001
BAUD_RATE = 9600

try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    print(f"Virtual Arduino on {SERIAL_PORT}")

    while True:
        # 生成模拟传感器数据（随机数模拟实际传感器）
        sensor_value = random.randint(200, 800)
        ser.write(f"SENSOR:{sensor_value}\n".encode("utf-8"))
        time.sleep(0.1)  # 模拟传感器数据更新频率

        # 监听 Python GUI 发送的电机控制指令
        if ser.in_waiting:
            command = ser.readline().decode('utf-8').strip()
            if command.startswith("MOTOR:"):
                motor_speed = command.split(":")[1]
                print(f"收到电机指令: {motor_speed}")

except serial.SerialException:
    print(f"无法打开虚拟串口 {SERIAL_PORT}，请确认端口是否正确。")
