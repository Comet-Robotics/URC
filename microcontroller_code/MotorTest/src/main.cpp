

// #include "../setup.cpp"

#include <Arduino.h>
#include "encoder_driver.h"
#include "pwm_driver.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#define LED 8
#define SLOWDOWN 26
#define CMD_VEL_TOPIC "/cmd_vel"
#define RPM_TOPIC "/rpm"
#define PWM_TOPIC "/pwm"
#define CMD_VEL_TIMEOUT_MS 5000

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED,!digitalRead(LED)); delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

// Variables for speed calculation
unsigned long lastTime = 0;
long lastPulseCount[MOTOR_COUNT] = {0};
int pwmValue = 0;
int target_PWM = 0;
bool leftDirection = true; // true for forward, false for reverse
bool rightDirection = true; // true for forward, false for reverse

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
std_msgs__msg__Float32MultiArray pwm_msg; // Equates to std_msgs::msg::Float32MultiArray

const unsigned long RPM_PUB_INTERVAL = 100; // Publish RPM every 100ms

unsigned long currentCMDVelTime = 0;
unsigned long lastCMDVelTime = 0;

// Function that is called when a new cmd_vel message is received
void cmd_vel_callback(const void * msgin) {
  digitalWrite(LED, !digitalRead(LED)); // Toggle built-in LED for visual feedback
  delay(100);
  digitalWrite(LED, !digitalRead(LED)); // Toggle back


  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  
  // TODO: Support turning

  // Extract linear velocity (x) and map to PWM
  // Assuming x velocity in range [-1.0, 1.0] maps to PWM [0, 255]
  float linear_vel = msg->linear.x;
  
  // Clamp to [-1.0, 1.0] range
  if (linear_vel > 1.0f) linear_vel = 1.0f;
  if (linear_vel < -1.0f) linear_vel = -1.0f;
  
  // TODO: Allow reversing
  // Map to PWM value
  target_PWM = (int)(linear_vel * PWM_MAX_VALUE);

  if(target_PWM < 0) { target_PWM = -target_PWM; } // Ensure positive
  
}

// Function that is called every RPM_PUB_INTERVAL milliseconds to publish the current RPM
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if(timer == NULL) {
    return;
  }

  // TODO: Adjust this to publish RPM for all motors

  /*
    ROBOT ORIENTATION
          FRONT
      MOTOR1  MOTOR2
      MOTOR3  MOTOR4 
      MOTOR5  MOTOR6  (6WD Ackermann Rover system)  
          BACK
  */
  // Calculate motor RPM
  unsigned long currentTime = millis();

  long currentPulseCount[MOTOR_COUNT];
  long pulseDiff[MOTOR_COUNT];

  float rpm[MOTOR_COUNT];
  
  for (int i = 0; i < MOTOR_COUNT; i++) {
    currentPulseCount[i] = readMotorPulses(static_cast<MotorID>(i));
    pulseDiff[i] = currentPulseCount[i] - lastPulseCount[i];
    float timeInterval = (currentTime - lastTime) / 1000.0; // Convert to seconds
    float pulsesPerSecond = pulseDiff[i] / timeInterval;
    rpm[i] = (pulsesPerSecond * 2.0 * 60.0) / PULSES_PER_REVOLUTION;
    lastPulseCount[i] = currentPulseCount[i];
  }

    // long pulseDiff = currentPulseCount - lastPulseCount;
    // float timeInterval = (currentTime - lastTime) / 1000.0;

    // float pulsesPerSecond = pulseDiff / timeInterval;
    // float rpm = (pulsesPerSecond * 2.0 * 60.0) / PULSES_PER_REVOLUTION;

  // Publish RPM data
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rpm_msg.data.data[i] = rpm[i];
  }
  rcl_publish(&rpm_pub, &rpm_msg, NULL);



  lastTime = currentTime;
  
  // Gradually adjust PWM towards target_PWM
  if (pwmValue < target_PWM) {
    pwmValue += 5; // Ramp up speed
    if (pwmValue > target_PWM) {
      pwmValue = target_PWM; // Don't exceed target
    }
  } else if (pwmValue > target_PWM) {
    pwmValue -= 5; // Ramp down speed
    if (pwmValue < target_PWM) {
      pwmValue = target_PWM; // Don't go below target
    }
  }

  // Publish PWM data
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm_msg.data.data[i] = pwmValue;
  }
  rcl_publish(&pwm_pub, &pwm_msg, NULL);

  // Set PWM for all motors
  for(int i = 0; i < MOTOR_COUNT; i++) {
    setMotorPWM(static_cast<MotorID>(i), pwmValue);
  }
}


void setup() {

  Serial.begin(115200);
  delay(2000); // Wait for serial to initialize
  set_microros_serial_transports(Serial);
  delay(2000);



  pinMode(LED, OUTPUT);

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

  RCCHECK(rclc_publisher_init_default(
    &pwm_pub,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    PWM_TOPIC));
  
  // Create timer for publishing RPM
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(RPM_PUB_INTERVAL),
      timer_callback));
  


  // Create executor
  executor = rclc_executor_get_zero_initialized_executor();
  // vvvvvvvvvvvv THIS MUST MATCH THE NUMBER OF THINGS THE EXECUTOR NEEDS TO KEEP TRACK OF (subscriptions, timers, etc.)
  int num_handles = 2; // 1 for cmd_vel subscription, 1 for timer
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  // Add subscription to executor
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

  
  lastTime = millis();

}

void loop() {

  // Spin the executor to handle incoming messages and timer events
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
}