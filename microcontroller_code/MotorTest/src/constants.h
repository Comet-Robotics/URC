#ifndef PINS_N_CONSTANTS_H
#define PINS_N_CONSTANTS_H

#define PULSES_PER_REVOLUTION 90  // Adjust based on your motor's pole pairs and phases
#define MOTOR_MAX_RPM 500  // Maximum RPM of the motor
#define PWM_MAX_VALUE 120  // Pi Pico default analogWrite range (0-255). Can increase with analogWriteResolution()


//if motor count changes go to encoder_driver and add or remove isr functions accordingly
#define MOTOR_COUNT 6 

/*
ROBOT ORIENTATION
         FRONT
    MOTOR1  MOTOR2  (2WD/DIFF DRIVE)
    MOTOR3  MOTOR4  (4WD/MECANUM)
    MOTOR5  MOTOR6  (6WD Ackermann Rover system)  
         BACK
*/

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

enum Group {
    LEFT,
    RIGHT,
    ALL
};

enum MotorID {
    FRONT_LEFT_MOTOR = 0,
    FRONT_RIGHT_MOTOR = 1,
    MIDDLE_LEFT_MOTOR = 2,
    MIDDLE_RIGHT_MOTOR = 3,
    REAR_LEFT_MOTOR = 4,
    REAR_RIGHT_MOTOR = 5,
};

//todo: update pin assignments for each motor
//arrays define pin assignments for each motor from front left to rear right
extern int MOTOR_ESC_SPEED_PINS[MOTOR_COUNT]; //encoder pins
extern int MOTOR_PWM_PINS[MOTOR_COUNT];
extern int MOTOR_DIRECTION_PINS[MOTOR_COUNT];
extern int MOTOR_BRAKE_PINS[MOTOR_COUNT]; // Brake control pins

#endif