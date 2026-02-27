// Microros controller that takes 4 angle commands and converts them to stepper motor steps
#include <Arduino.h>
#include "constants.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h> // For getting degrees from /steering_angles topic

#define STEERING_ANGLE_TOPIC "/steering_angles"

#define STEERING_TIMEOUT_MS 5000
#define TIME_BETWEEN_STEPS_MS 2

#ifndef LED_BUILTIN
#define LED 8
#else
#define LED LED_BUILTIN
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED,!digitalRead(LED)); delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

unsigned long lastSteeringCmdTime = 0;
unsigned long currentSteeringCmdTime = 0;

unsigned long lastTime = 0;

Motor motors[MOTOR_COUNT];


rcl_subscription_t steering_sub;
rcl_node_t node;
rcl_allocator_t allocator;

rclc_support_t support;
rclc_executor_t executor;

std_msgs__msg__Float32MultiArray steering_msg;

void steering_callback(const void * msgin) {
    digitalWrite(LED, !digitalRead(LED));

    unsigned long now = millis();
    lastSteeringCmdTime = now;

    const std_msgs__msg__Float32MultiArray* msg =
        (const std_msgs__msg__Float32MultiArray*) msgin;

    if (msg->data.size < MOTOR_COUNT) return;

    for(int i = 0; i < MOTOR_COUNT; i++) {
        motors[i].setDeg(msg->data.data[i]);
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 5000) {}
    delay(1000);
    set_microros_serial_transports(Serial);


    // Setup motors
    for(int i = 0; i < MOTOR_COUNT; i++) {
        motors[i] = Motor(MOTOR_PINS[i][0], MOTOR_PINS[i][1], MOTOR_PINS[i][2], MOTOR_PINS[i][3], MOTOR_PINS[i][4]);
        motors[i].setup();
    }

    // Initialize allocator
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

    // Create node
    RCCHECK(rclc_node_init_default(
        &node,
        "steering_controller_node", 
        "", 
        &support));

    // Create subscriber for steering angles
    RCCHECK(rclc_subscription_init_default(
        &steering_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
        STEERING_ANGLE_TOPIC));
    
    // Create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

    // Initialze steering message
    steering_msg.data.data = (float*) malloc(sizeof(float) * MOTOR_COUNT);
    steering_msg.data.size = 0;
    steering_msg.data.capacity = MOTOR_COUNT;

    for(int i = 0; i < MOTOR_COUNT; i++)
        steering_msg.data.data[i] = 0.0f;

    // Add subscription to executor
    RCCHECK(rclc_executor_add_subscription(
        &executor,
        &steering_sub,
        &steering_msg,
        &steering_callback,
        ON_NEW_DATA));

    
    digitalWrite(LED, HIGH); // Turn on LED to indicate setup is complete
    delay(50);
    digitalWrite(LED, LOW);
}

void loop()
{
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));

    // Check for steering command timeout
    if (millis() - lastSteeringCmdTime > STEERING_TIMEOUT_MS) {
        // if no command has arrived recently, force targets to zero
        for(int i = 0; i < MOTOR_COUNT; i++) {
            motors[i].setDeg(0);
        }
    }
    if(millis() - lastTime >= TIME_BETWEEN_STEPS_MS) {
        for(int i = 0; i < MOTOR_COUNT; i++) {
            motors[i].takeStep();
        }
        lastTime = millis();
    }
}