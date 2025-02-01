#include <Servo.h>           // Include Servo library for servo motor control
#include <AccelStepper.h>    // Include AccelStepper library for stepper motor control

// === Sensor Pin Definitions ===
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2

// === Servo Motor (Pin 9) ===
Servo myServo;
int servoAngle = 90;  // Default angle

// === Stepper Motor Definitions ===
#define STEP_PIN 4
#define DIR_PIN 5
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
int stepperSpeed = 200;  // Default speed

// === DC Motor with Encoder Definitions ===
#define DC_MOTOR_PWM 10   // PWM control pin
#define DC_MOTOR_DIR 11   // Direction control pin
#define ENCODER_A 2       // Encoder channel A
#define ENCODER_B 3       // Encoder channel B
volatile long encoderCount = 0;  // Encoder count
int motorSpeed = 0;  // PWM speed (0-255)

// === Interrupt for Encoder ===
void encoderISR() {
    if (digitalRead(ENCODER_B) == HIGH) {
        encoderCount++;
    } else {
        encoderCount--;
    }
}

void setup() {
    Serial.begin(115200);  // Initialize serial communication

    // === Sensor Initialization ===
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);

    // === Initialize Servo Motor ===
    myServo.attach(9);
    myServo.write(servoAngle);  // Set initial angle

    // === Initialize Stepper Motor ===
    stepper.setMaxSpeed(1000);  // Set max speed
    stepper.setAcceleration(500);  // Set acceleration

    // === Initialize DC Motor ===
    pinMode(DC_MOTOR_PWM, OUTPUT);
    pinMode(DC_MOTOR_DIR, OUTPUT);
    pinMode(ENCODER_A, INPUT_PULLUP);
    pinMode(ENCODER_B, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(ENCODER_A), encoderISR, CHANGE);  // Attach interrupt for encoder

    // Stop all motors initially
    analogWrite(DC_MOTOR_PWM, 0);
    digitalWrite(DC_MOTOR_DIR, LOW);
}

void loop() {
    // === Read Sensor Data ===
    int sensor1Value = analogRead(SENSOR_1);
    int sensor2Value = analogRead(SENSOR_2);
    int sensor3Value = analogRead(SENSOR_3);

    // === Send Sensor Data to Python GUI ===
    Serial.print("SENSOR:");
    Serial.print(sensor1Value);
    Serial.print(",");
    Serial.print(sensor2Value);
    Serial.print(",");
    Serial.println(sensor3Value);

    // === Listen for Motor Control Commands from Python GUI ===
    while (Serial.available()) {
        String command = Serial.readStringUntil('\n');  // Read command
        command.trim();  // Trim whitespace or newline

        // === Control Servo Motor ===
        if (command.startsWith("MOTOR1:")) {
            servoAngle = command.substring(7).toInt();  // Extract value
            servoAngle = constrain(servoAngle, 0, 180);  // Limit angle range
            myServo.write(servoAngle);
        }

        // === Control Stepper Motor ===
        else if (command.startsWith("MOTOR2:")) {
            stepperSpeed = command.substring(7).toInt();  // Extract stepper motor speed
            stepperSpeed = constrain(stepperSpeed, -500, 500);  // Limit speed range
            stepper.setSpeed(stepperSpeed);
        }

        // === Control DC Motor with Encoder ===
        else if (command.startsWith("MOTOR3:")) {
            motorSpeed = command.substring(7).toInt();  // Extract DC motor speed
            motorSpeed = constrain(motorSpeed, -255, 255);  // Limit PWM range
            
            if (motorSpeed >= 0) {
                digitalWrite(DC_MOTOR_DIR, HIGH);
                analogWrite(DC_MOTOR_PWM, motorSpeed);
            } else {
                digitalWrite(DC_MOTOR_DIR, LOW);
                analogWrite(DC_MOTOR_PWM, -motorSpeed);
            }
        }
    }

    // === Run Stepper Motor ===
    stepper.runSpeed();

    // === Send Encoder Data to Python GUI ===
    Serial.print("ENCODER:");
    Serial.println(encoderCount);

    delay(50);  // 50ms sampling interval
}
