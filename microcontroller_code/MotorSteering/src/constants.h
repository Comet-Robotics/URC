#include <Arduino.h>
#ifndef PINS_N_CONSTANTS_H
#define PINS_N_CONSTANTS_H

#define PULSES_PER_REVOLUTION 90  // Adjust based on your motor's pole pairs and phases
#define MOTOR_MAX_RPM 500  // Maximum RPM of the motor
#define PWM_MAX_VALUE 120  // Pi Pico default analogWrite range (0-255). Can increase with analogWriteResolution()


// Code assumes cmd_vel will send values in the range of [-MAX_CMD_VEL, MAX_CMD_VEL].
#define MAX_CMD_VEL 1.0f 



//if motor count changes go to encoder_driver and add or remove isr functions accordingly
#define MOTOR_COUNT 4 
#define STEERING_MOTOR_COUNT 4

/*
ROBOT ORIENTATION
         FRONT                                      <-----
    MOTOR1  MOTOR2  (2WD/DIFF DRIVE)                      |  
    MOTOR3  MOTOR4  (4WD/MECANUM)                         | --- Aprox 0.83 meters (wheelbase) 
    MOTOR5  MOTOR6  (6WD Ackermann Rover system)          |  
         BACK                                       <-----
    ^------------^
           | 
           Approx 0.78 meters (track width does very between wheel pairs) 
*/

#define TRACK_WIDTH 0.78f    
#define WHEELBASE 0.83f

struct SteeringMotor{
    // Each motor has a step per rev of 200, and a gearbox of 14:1 for an steps per rev of 2800 on the output.
    int AIN1;
    int AIN2;
    int BIN1;
    int BIN2;
    int sleepPin;

    int targetStep = 0;
    int step = 0;

    int stepsTaken = 0; // Mental image of where we are

    SteeringMotor() {}
    SteeringMotor(int a1, int a2, int b1, int b2, int nSleep)
      : AIN1(a1), AIN2(a2), BIN1(b1), BIN2(b2), sleepPin(nSleep),
        targetStep(0), step(0), stepsTaken(0) {}

    void setup(){
        pinMode(AIN1, OUTPUT);
        pinMode(AIN2, OUTPUT);
        pinMode(BIN1, OUTPUT);
        pinMode(BIN2, OUTPUT);
        pinMode(sleepPin, OUTPUT);

        digitalWrite(sleepPin, HIGH); // Wake up the motor driver
    }

    void setStep(int step) {
        switch(step) {
            case 0: digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
                    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); break;

            case 1: digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
                    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW); break;

            case 2: digitalWrite(AIN1, LOW); digitalWrite(AIN2, HIGH);
                    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); break;

            case 3: digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
                    digitalWrite(BIN1, LOW); digitalWrite(BIN2, HIGH); break;
        }
    }

    void setDeg(int deg)
    {
        int stepsPerRev = 2800; // 200 steps/rev * 14:1 gearbox
        targetStep = (deg * stepsPerRev) / 360; // Convert degrees to steps
    }

    void takeStep()
    {
        // FIXME: This more or less worked, but double check that this is correct
        if (stepsTaken > targetStep) {
            // moving forward
            step = (step + 1) % 4;
            setStep(step);
            stepsTaken--;
        } else if (stepsTaken < targetStep) {
            // moving backward
            step = (step - 1 + 4) % 4;
            setStep(step);
            stepsTaken++;
        }
    }

};

enum MotorDirection {
    FORWARD,
    REVERSE
};

enum MotorState {
    RUNNING,
    STOPPING,
    WAITING_FOR_STOP,
    NEUTRAL,
    CHANGING_DIRECTION
};

enum Heading {
    FORWARD_DIR,
    BACKWARD_DIR,
    LEFT_DIR,
    RIGHT_DIR
};

enum Group {
    LEFT,
    RIGHT,
    ALL
};

enum MotorID {
    FRONT_LEFT_MOTOR = 0,
    FRONT_RIGHT_MOTOR = 1,
    // MIDDLE_LEFT_MOTOR = 2,
    // MIDDLE_RIGHT_MOTOR = 3,
    REAR_LEFT_MOTOR = 2,
    REAR_RIGHT_MOTOR = 3,
};

//todo: update pin assignments for each motor
//arrays define pin assignments for each motor from front left to rear right
extern int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT]; //encoder pins
extern int MOTOR_PWM_PINS[MOTOR_COUNT];
extern int MOTOR_DIRECTION_PINS[MOTOR_COUNT];
extern int MOTOR_BRAKE_PINS[MOTOR_COUNT]; // Brake control pins
extern int STEERING_MOTOR_PINS[STEERING_MOTOR_COUNT][5]; // 2 for A, 2 for B, 1 for sleep

#endif