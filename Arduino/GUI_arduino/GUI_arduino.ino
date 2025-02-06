#include <Servo.h>           // Include Servo library for servo motor control
#include <AccelStepper.h>    // Include AccelStepper library for stepper motor control

// === Sensor Pin Definitions ===
#define SENSOR_1 A0
#define SENSOR_2 A1
#define SENSOR_3 A2

// === Servo Motor ===
Servo myServo;
#define SERVO_PIN 13  // Pin for the servo control
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
// // Pin Definition
// const byte interruptPinA = 2;
// const byte interruptPinB = 3;
// volatile long EncoderCount = 0;
// const byte PWMPin = 6;
// const byte DirPin1 = 7;
// const byte DirPin2 = 4;

// // PID controller
// float kp = 0.02;
// float ki = 0.00015 ;
// float kd = 0;

// unsigned long t;
// unsigned long t_prev = 0;

// volatile unsigned long count = 0;
// unsigned long count_prev = 0;

// float Theta, RPM, RPM_d = 50;
// float Theta_prev = 0;
// int dt;
// float RPM_max = 230;

// #define pi 3.1416
// float Vmax = 6;
// float Vmin = -6;
// float V = 0.1;
// float e, e_prev = 0, inte, inte_prev = 0; 

// DC_small
#define IN1 2   // Motor control signal 1 (PWM output)
#define IN2 3  // Motor control signal 2 (PWM output)

float DC_run = 0.0;

void motorForward(int frequency) {
    tone(IN1, frequency);  // Generate PWM signal on IN1
    digitalWrite(IN2, LOW);
}
void motorStop() {
    noTone(IN1);
    noTone(IN2);
}


// // === Interrupt for Encoder ===
// void ISR_EncoderA() {
//   bool PinB = digitalRead(interruptPinB);
//   bool PinA = digitalRead(interruptPinA);

//   if (PinB == LOW) {
//     if (PinA == HIGH) {
//       EncoderCount++;
//     }
//     else {
//       EncoderCount--;
//     }
//   }

//   else {
//     if (PinA == HIGH) {
//       EncoderCount--;
//     }
//     else {
//       EncoderCount++;
//     }
//   }
// }

// void ISR_EncoderB() {
//   bool PinB = digitalRead(interruptPinA);
//   bool PinA = digitalRead(interruptPinB);

//   if (PinA == LOW) {
//     if (PinB == HIGH) {
//       EncoderCount--;
//     }
//     else {
//       EncoderCount++;
//     }
//   }

//   else {
//     if (PinB == HIGH) {
//       EncoderCount++;
//     }
//     else {
//       EncoderCount--;
//     }
//   }
// }

// float sign(float x) {
//   if (x > 0) {
//     return 1;
//   } else if (x < 0) {
//     return -1;
//   } else {
//     return 0;
//   }
// }

// //**DC Motor Driver Functions**
// void WriteDriverVoltage(float V, float Vmax) {
//   int PWMval = int(255 * abs(V) / Vmax);
//   if (PWMval > 255) {
//     PWMval = 255;
//   }
//   if (V > 0) {
//     digitalWrite(DirPin1, HIGH);
//     digitalWrite(DirPin2, LOW);
//   }
//   else if (V < 0) {
//     digitalWrite(DirPin1, LOW);
//     digitalWrite(DirPin2, HIGH);
//   }
//   else {
//     digitalWrite(DirPin1, LOW);
//     digitalWrite(DirPin2, LOW);
//   }
//   analogWrite(PWMPin, PWMval);
// }

// ISR(TIMER2_COMPA_vect) {
//   count++;
//   //Serial.print(count * 0.05); Serial.print(" \t");
// }


// Senser transfer function
// Sensor 1 (Potentiometer): digit to resistance value(Ohm)
float R_senser1 = 10000.0;
float transfer_sensor1(int input){
  int MAX = 1024;
  int MIN = 0;
  float output;
  // Output the Resistance of the Potentiometer
  output = input * R_senser1 / (MAX - MIN);
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
    Serial.begin(115200);  // Initialize serial communication

    // === Sensor Initialization ===
    pinMode(SENSOR_1, INPUT);
    pinMode(SENSOR_2, INPUT);
    pinMode(SENSOR_3, INPUT);

    // === Initialize Servo Motor ===
    myServo.attach(SERVO_PIN);
    myServo.write(servoAngle);  // Set initial angle

    // === Initialize Stepper Motor ===
    stepper.setMaxSpeed(1000.0);  // Set max speed
    stepper.setAcceleration(500.0);  // Set acceleration
    // Move the motor to the initial position
    stepper.moveTo(stepperPosition); // One full revolution for 28BYJ-48

    // === Initialize DC Motor ===
    // pinMode(interruptPinA, INPUT_PULLUP);
    // pinMode(interruptPinB, INPUT_PULLUP);
    // attachInterrupt(digitalPinToInterrupt(interruptPinA), ISR_EncoderA, CHANGE);
    // attachInterrupt(digitalPinToInterrupt(interruptPinB), ISR_EncoderB, CHANGE);
    // pinMode(DirPin1, OUTPUT);
    // pinMode(DirPin2, OUTPUT);

    // cli();
    // TCCR2A = 0;
    // TCCR2B = 0;
    // TCNT2 = 0;
    // OCR2A = 249; //Prescaler = 64
    // TCCR2A |= (1 << WGM12);
    // TCCR2B |= (1 << CS22);
    // TIMSK2 |= (1 << OCIE2A);
    // sei();

    // === Initialize DC_s Motor ===
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);

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
          //RPM_d = command.substring(7).toInt();  // Extract DC motor desire speed
          //RPM_d = constrain(RPM_d, -RPM_max, RPM_max);  // Limit RPM range
          DC_run = command.substring(7).toInt();
          DC_run = constrain(DC_run, 0, 500);
        }
    }

    // === Run Stepper Motor ===
    // If the motor is at the target position, move it in the opposite direction
    stepper.run();

    // == Run the DC motor
    motorForward(DC_run);
    // if (count > count_prev) {
    //   t = millis();
    //   Theta = EncoderCount / 900.0;
    //   dt = (t - t_prev);
    //   RPM = (Theta - Theta_prev) / (dt / 1000.0) * 60;
    //   e = RPM_d - RPM;
    //   inte = inte_prev + (dt * (e + e_prev) / 2);
    //   V = kp * e + ki * inte + (kd * (e - e_prev) / dt) ;
    //   if (V > Vmax) {
    //     V = Vmax;
    //     inte = inte_prev;
    //   }
    //   if (V < Vmin) {
    //     V = Vmin;
    //     inte = inte_prev;
    //   }
    //   WriteDriverVoltage(V, Vmax);
    //   Theta_prev = Theta;
    //   count_prev = count;
    //   t_prev = t;
    //   inte_prev = inte;
    //   e_prev = e;
    // }


    //delay(1);  // 1ms sampling interval
}