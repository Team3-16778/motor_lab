# motor_lab
Codes for the motor lab (Arduino and python)

## GUI
The user interface for the motor lab.

### Python version
- Python 3.12.3

### Dependencies
- pyqt6
- pyserial
- matplotlib
- numpy

## Arduino

### Hardware
- Arduino Uno
- Sensors
    - Potentiometer
    - Photoresistor
    - Flux
    - Encoder(Integrated)
- Motors
    - Servo motor
    - Stepper motor
    - DC motor with encoder

### Debug
To run without Arduino, use the `fake_arduino.py` script.
(In mac, you should install `socat` by `brew install socat` and run `socat -d -d pty,raw,echo=0 pty,raw,echo=0`.)
