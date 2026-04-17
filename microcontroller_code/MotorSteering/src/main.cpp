#include <Arduino.h>
#include "constants.h"
#include "encoder_driver.h"
#include "pwm_driver.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 8
#else
#define LED LED_BUILTIN
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED,!digitalRead(LED)); delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

#define LOOP_STEP_MS 10
#define RAMP_STEP 3
#define STOP_THRESHOLD 1.0 // RPM threshold to consider motor stopped

#define CMD_VEL_TOPIC "/cmd_vel"
#define RPM_TOPIC "/rpm"

#define CMD_VEL_TIMEOUT_MS 5000

#define PULSES_PER_REVOLUTION 90  

// Micro-ROS variables
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t rpm_pub;
rcl_publisher_t pwm_pub;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_timer_t timer;

rclc_support_t support;
rclc_executor_t executor;

// Micro-ROS messages
geometry_msgs__msg__Twist cmd_vel_msg; // Equates to geometry_msgs::msg::Twist
std_msgs__msg__Float32MultiArray rpm_msg; // Equates to std_msgs::msg::Float32MultiArray
std_msgs__msg__Float32MultiArray pwm_msg;

unsigned long lastCMDVelTime = 0;
unsigned long RPM_PUB_INTERVAL = 100; // Publish RPM every 100ms


struct Motor{
    MotorID id;
    unsigned long lastPulseTime = 0;

    int targetPWM = 0;
    int currentPWM = 0;

    int fakeRPM = 0; // For testing without encoders, remove when encoders are working

    bool forward = true;
    bool targetForward = true;

    int lastPulseCount = 0;

    Motor(MotorID motorID) {id = motorID;}

    // Ramp towards target PWM, return true if we are at target
    bool step(int rampStep) { 
        // Debug:
        fakeRPM = currentPWM;
        if(forward != targetForward) {
            if(currentPWM > 0) {
                currentPWM -= rampStep; // Ramp down to 0 before changing direction
                if(currentPWM < 0) currentPWM = 0;
            } 
            // Switching the direction of the motor will be handled in the main logic loop after checking rpm is 0
            return false;
        }
        
        if(targetPWM == currentPWM) {
            return true; // We are at target
        }

        if(currentPWM < targetPWM) {
            currentPWM += rampStep; // Ramp up speed
            if(currentPWM > targetPWM) currentPWM = targetPWM;
        } else {
            currentPWM -= rampStep; // Ramp down speed
            if(currentPWM < targetPWM) currentPWM = targetPWM;
        }
    

        return false;
    } 

    int getRPM()
    {
        unsigned long currentTime = millis();

        // Calculate RPM based on time between pulses
        unsigned long timeDiff = currentTime - lastPulseTime;
        if(timeDiff == 0) return 0; // Avoid division by zero

        int currentPulseCount = readMotorPulses(id);
        int pulseDiff = currentPulseCount - lastPulseCount;
        lastPulseCount = currentPulseCount;

        float pulsesPerSecond = pulseDiff / (timeDiff / 1000.0);
        float rpm = (pulsesPerSecond * 60.0) / PULSES_PER_REVOLUTION;

        lastPulseTime = currentTime;
        return rpm;

        // Debug
        // return fakeRPM;
    }

    int getPWM() {
        return currentPWM;
    }
    bool isForward() {
        return forward;
    }

    
};

Motor motors[MOTOR_COUNT] = {Motor(FRONT_LEFT_MOTOR), Motor(FRONT_RIGHT_MOTOR), Motor(REAR_LEFT_MOTOR), Motor(REAR_RIGHT_MOTOR)};


// Function that handles incoming cmd_vel messages
void cmd_vel_callback(const void * msgin)
{
    // Simple Skid Steering:
    //
    //    Steering via changing the direction of motors on one side.
    //    ie: to turn left, set left motors to reverse and right motors to forward.
    //
    //
    //    Left_PWM = v - (w * TRACK_WIDTH / 2) 
    //    Right_PWM = v + (w * TRACK_WIDTH / 2)
    //
    //    where v is the linear velocity from cmd_vel (linear.x) and w is the angular velocity from cmd_vel (angular.z).

    lastCMDVelTime = millis();

    const geometry_msgs__msg__Twist* msg = (const geometry_msgs__msg__Twist*) msgin;

    float v = msg->linear.x;
    float w = msg->angular.z;

    float left_pwm, right_pwm;

    left_pwm = v - (w * (TRACK_WIDTH / 2.0f));
    right_pwm = v + (w * (TRACK_WIDTH / 2.0f));

    // Remap this to -MAX_PWM to MAX_PWM
    left_pwm = (left_pwm / MAX_CMD_VEL) * PWM_MAX_VALUE;
    right_pwm = (right_pwm / MAX_CMD_VEL) * PWM_MAX_VALUE;

    // Update motor targets
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if(i % 2 == 0) { // Left motors
            motors[i].targetPWM = abs(static_cast<int>(left_pwm));
            motors[i].targetForward = left_pwm >= 0;
        }
        else { // Right motors
            motors[i].targetPWM = fabs(static_cast<int>(right_pwm));
            motors[i].targetForward = right_pwm >= 0;
        }
    }
}

// Timer that publishes RPM / Main logic loop
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    if(timer == NULL) {
        return;
    }

    // Find the error of each motor so that we can scale their ramp step so that they reach their target at the same time
    int errors[MOTOR_COUNT] = {0};
    for(int i = 0; i < MOTOR_COUNT; i++) {
        // Find error such that
        // - If the motor is in the correct direction, error is just the difference in PWM
        // - If the motor is in the wrong direction, error is the sum of the current PWM 
        //   and target PWM since we need to ramp down to 0 before ramping up in the other direction
        if(motors[i].isForward() == motors[i].targetForward) {
            errors[i] = abs(motors[i].targetPWM - motors[i].currentPWM);
        }
        else {
            errors[i] = motors[i].targetPWM + motors[i].currentPWM;
        }
    }

    // Determine the maximum error, which we will scale against
    int maxError = 0;
    for(int i = 0; i < MOTOR_COUNT; i++) {
        if(errors[i] > maxError) {
            maxError = errors[i];
        }
    }

    // Handle each motor
    for(int i = 0; i < MOTOR_COUNT; i++) {
        
        // Update RPM
        float rpm = motors[i].getRPM();

        // Check if we need to change direction
        if(motors[i].forward != motors[i].targetForward) {
            if(fabs(rpm) < STOP_THRESHOLD) { // Check if RPM is considered stopped
                motors[i].forward = motors[i].targetForward;
                setMotorDirection(motors[i].id, motors[i].forward ? FORWARD : REVERSE);
            }
        }

        // Add rpm to message
        rpm_msg.data.data[i] = rpm;


        // Handle PWM
        float scale = maxError > 0 ? ((float)errors[i] / maxError) : 0; // Scale ramp step based on error, if maxError is 0 then all motors are at target so scale doesn't matter

        if(!motors[i].step(RAMP_STEP * scale)) { // Change PWM if we are not at target
            setMotorPWM(motors[i].id, motors[i].getPWM());
        }
    }
    

    // Publish RPM data
    rcl_publish(&rpm_pub, &rpm_msg, NULL);

    // Check if cmd_vel message has timed out
    if(millis() - lastCMDVelTime > CMD_VEL_TIMEOUT_MS) {
        for(int i = 0; i < MOTOR_COUNT; i++) {
            motors[i].targetPWM = 0;
            motors[i].targetForward = true;
        }
    }

    // Publish PWM data
    for(int i = 0; i < MOTOR_COUNT; i++) {
        pwm_msg.data.data[i] = motors[i].getPWM() * (motors[i].isForward() ? 1 : -1); // Publish negative PWM for reverse direction for easier debugging
    }
    rcl_publish(&pwm_pub, &pwm_msg, NULL);

    // Debug, set the LED to on if the left motors are reversing --  FIXME: Doesn't work for some reason
    if(motors[0].isForward() == false) {
        digitalWrite(LED, HIGH);
    }
    else {
        digitalWrite(LED, LOW);
    }

    
}

void setup() {
    Serial.begin(115200);
    pinMode(LED, OUTPUT);
    while (!Serial && millis() < 5000) {}
    delay(1000);

    set_microros_serial_transports(Serial);

    

    // Initialize driven motors
    for(int i = 0; i < MOTOR_COUNT; i++) {
        pinMode(MOTOR_PWM_PINS[i], OUTPUT);
        analogWrite(MOTOR_PWM_PINS[i], 0); // Initialize to 0
    }


    initMotorSpeedReader();

    // Initialize allocator
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));

    // Create cmd_vel subscriber
    RCCHECK(rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        CMD_VEL_TOPIC));

    // Create rpm publisher
    RCCHECK(rclc_publisher_init_default(
        &rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        RPM_TOPIC));

    // Create pwm publisher
    RCCHECK(rclc_publisher_init_default(
        &pwm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        "/pwm"));

    // Create timer for publishing RPM
    RCCHECK(rclc_timer_init_default(
        &timer,
        &support,
        RCL_MS_TO_NS(RPM_PUB_INTERVAL),
        timer_callback));


    // Create executor
    executor = rclc_executor_get_zero_initialized_executor();
    RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));

    // Add cmd_vel subscription to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor, 
        &cmd_vel_sub, 
        &cmd_vel_msg,
        &cmd_vel_callback, 
        ON_NEW_DATA));  

    // Add timer to executor
    RCCHECK(rclc_executor_add_timer(
        &executor, 
        &timer));



    // Initialize RPM message
    rpm_msg.data.capacity = MOTOR_COUNT;
    rpm_msg.data.size = MOTOR_COUNT;
    rpm_msg.data.data = (float *)malloc(MOTOR_COUNT * sizeof(float));

    // Initialize PWM message
    pwm_msg.data.capacity = MOTOR_COUNT;
    pwm_msg.data.size = MOTOR_COUNT;
    pwm_msg.data.data = (float *)malloc(MOTOR_COUNT * sizeof(float));

    lastCMDVelTime = millis();

    for(int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].targetPWM = 0;
        motors[i].currentPWM = 0;
        motors[i].forward = true;
        motors[i].targetForward = true;

        motors[i].lastPulseTime = millis();
    }
    
}

void loop() {

    // Spin the executor 
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(LOOP_STEP_MS));

}