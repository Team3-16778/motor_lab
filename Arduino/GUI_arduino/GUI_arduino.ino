#include <Servo.h>           // Include Servo library for servo motor control
#include <AccelStepper.h>    // Include AccelStepper library for stepper motor control

// === Sensor Pin Definitions ===
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2

// === Servo Motor ===
Servo myServo;
#define SERVO_PIN 4  // Pin for the servo control
int servoAngle = 90;  // Default angle

// === Stepper Motor Definitions ===
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define HALFSTEP 8
AccelStepper stepper(HALFSTEP, IN1, IN3, IN2, IN4);
int stepperPosition = 0;  // Default Position of the stepper

// === DC Motor with Encoder Definitions ===
#define DC_MOTOR_PWM 12   // PWM control pin
#define DC_MOTOR_DIR 13   // Direction control pin
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

// Senser transfer function
// Sensor 1 (Potentiometer): digit to resistance value(Ohm)
float R_senser1 = 10000.0
float transfer_sensor1(int input){
  int MAX = 1024;
  int MIN = 0;
  float output;
  // Output the Resistance of the Potentiometer
  output = input / (MAX - MIN) * R_senser1;
  return output;
}
// Sensor 2 (Photoresistor): digit to percentage of illumination
float transfer_sensor2(int input){
  int MIN = 450;
  int MAX = 850;
  input = constrain(input, MIN, MAX);
  float output = (float)(input - MIN) / (MAX - MIN) * 100.0;
  return output;
}
// Sensor 3 (Flex): digit to degrees
const float VCC = 5;
const float R_DIV = 47500.0; 
const float STRAIGHT_RESISTANCE = 163000.0; 
const float BEND_RESISTANCE = 500000.0;
float transfer_sensor3(int input){
  int flexADC = input;
  float flexV = flexADC * VCC / 1023.0;
  // Resistance(ohms)
  float flexR = R_DIV * (VCC / flexV - 1.0);
  // mapping the R to bending degree
  float angle = map(flexR, STRAIGHT_RESISTANCE, BEND_RESISTANCE,0, 90.0);
  return angle;
}

void setup() {
    Serial.begin(9600);  // Initialize serial communication

    // === Sensor Initialization ===
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);

    // === Initialize Servo Motor ===
    myServo.attach(SERVO_PIN);
    myServo.write(servoAngle);  // Set initial angle

    // === Initialize Stepper Motor ===
    stepper.setMaxSpeed(1000);  // Set max speed
    stepper.setAcceleration(500);  // Set acceleration
    // Move the motor to the initial position
    stepper.moveTo(stepperPosition); // One full revolution for 28BYJ-48

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
    Serial.print(transfer_sensor1(sensor1Value));
    Serial.print(",");
    Serial.print(transfer_sensor2(sensor2Value));
    Serial.print(",");
    Serial.println(transfer_sensor3(sensor3Value));

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
        if (command.startsWith("MOTOR2:")) {
            stepperPosition = command.substring(7).toInt();  // Extract stepper motor target position
            stepperPosition = constrain(stepperPosition, -2048, 2048);  // Limit target position
            stepper.moveTo(stepperPosition);
        }

        // === Control DC Motor with Encoder ===
        if (command.startsWith("MOTOR3:")) {
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
    stepper.run();

    // === Send Encoder Data to Python GUI ===
    // Serial.print("ENCODER:");
    // Serial.println(encoderCount);

    delay(50);  // 50ms sampling interval
}