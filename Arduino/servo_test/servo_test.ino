#include <Servo.h> // add servo library

Servo myservo;  // create servo object to control a servo

int angle;        // variable to read the value from the analog pin

void setup() {
  myservo.attach(9);
  Serial.begin(9600);  // attaches the servo on pin 9 to the servo object
}

float fun_for_sensor1(int input){
  int MAX = 1000;
  int MIN = 0;
  float output;
  output = input * 180.0 / (MAX - MIN);
  return output;
}
void loop() {
  int sensor1val = analogRead(A0);            // reads the value of the Degree D
  angle  = fun_for_sensor1(sensor1val);
  myservo.write(angle);                  // sets the servo position according  
  Serial.println(angle);
  delay(500);                           // waits for the servo to get there
}