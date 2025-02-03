// Include the AccelStepper library
#include <AccelStepper.h>

// Define the stepper motor connections and interface type
#define IN1 8
#define IN2 9
#define IN3 10
#define IN4 11
#define HALFSTEP 8

// Create an instance of AccelStepper with 8 steps sequence (half-step mode)
AccelStepper stepper(HALFSTEP, IN1, IN3, IN2, IN4);

void setup() {
    // Set the maximum speed and acceleration
    stepper.setMaxSpeed(1000.0);
    stepper.setAcceleration(500.0);
    // Move the motor to the initial position
    stepper.moveTo(2048); // One full revolution for 28BYJ-48
}

void loop() {
    // If the motor is at the target position, move it in the opposite direction
    // if (stepper.distanceToGo() == 0) {
    //     stepper.moveTo(-stepper.currentPosition());
    // }
    // Run the motor to the target position
    stepper.moveTo(-2048);
    stepper.run();
}
