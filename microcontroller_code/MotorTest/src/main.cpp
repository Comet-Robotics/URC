// 2/27/202 Combined SteeringController into here since the teensy is now controlling the Steppers

#include <Arduino.h>
#include "encoder_driver.h"
#include "pwm_driver.h"

#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>

#ifdef LED_BUILTIN
#define LED LED_BUILTIN
#else
#define LED 8
#endif

#define WHEELBASE 0.83f      // meters (front to rear axle)
#define TRACK_WIDTH 0.78f    // meters (left to right wheels; An approximation as each pair is a different distance apart)

#define SLOWDOWN 26

#ifndef RAMP_STEP
#define RAMP_STEP 1
#endif

#ifndef STOP_THRESHOLD
#define STOP_THRESHOLD 15.0 // RPM threshold to consider motor stopped
#endif

#define CMD_VEL_TOPIC "/cmd_vel"
#define RPM_TOPIC "/rpm"
#define PWM_TOPIC "/pwm"
#define CMD_VEL_TIMEOUT_MS 5000

#define STEERING_ANGLE_TOPIC "/steering_angles"
#define STEERING_TIMEOUT_MS 5000
#define TIME_BETWEEN_STEPS_MS 2


#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED,!digitalRead(LED)); delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

// Variables for speed calculation
unsigned long lastTime = 0;
long lastPulseCount[MOTOR_COUNT] = {0};
int pwmValue[MOTOR_COUNT] = {0};
int target_PWM[MOTOR_COUNT] = {0};
MotorDirection leftDirection = FORWARD; 
MotorDirection rightDirection = FORWARD; 
MotorState motorStates[MOTOR_COUNT] = {NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL, NEUTRAL};


unsigned long lastSteeringCmdTime = 0;

SteeringMotor motors[MOTOR_COUNT];

// Micro-ROS variables
rcl_subscription_t cmd_vel_sub;
rcl_subscription_t steering_sub;
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
std_msgs__msg__Float32MultiArray steering_msg;

const unsigned long RPM_PUB_INTERVAL = 100; // Publish RPM every 100ms

unsigned long currentCMDVelTime = 0;
unsigned long lastCMDVelTime = 0;

void setMotors(Group group, int PWM)
{
  // Set PWM for all motors in the specified group
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if ((group == LEFT && (i % 2 == 0)) || (group == RIGHT && (i % 2 == 1)) || (group == ALL)) {
      setMotorPWM(static_cast<MotorID>(i), abs(PWM));
    }
  }
}

void setBrake(MotorID motorID, int value)
{
  digitalWrite(MOTOR_BRAKE_PINS[motorID], value);
}

void setBrakes(Group group, int value)
{
  // Set brake state for all motors in the specified group
  for(int i = 0; i < MOTOR_COUNT; i++) {
    if ((group == LEFT && (i % 2 == 0)) || (group == RIGHT && (i % 2 == 1)) || (group == ALL)) {
      setBrake(static_cast<MotorID>(i), value);
    }
  }
}

// Motor Step, but we don't change directions
bool simpleMotorSetp(MotorID motorID, int targetPWM)
{
  if (pwmValue[motorID] < target_PWM[motorID]) {
    pwmValue[motorID] += RAMP_STEP; // Ramp up speed
    if (pwmValue[motorID] > target_PWM[motorID]) {
      pwmValue[motorID] = target_PWM[motorID]; // Don't exceed target
    }
  } else if (pwmValue[motorID] > target_PWM[motorID]) {
    pwmValue[motorID] -= RAMP_STEP; // Ramp down speed
    if (pwmValue[motorID] < target_PWM[motorID]) {
      pwmValue[motorID] = target_PWM[motorID]; // Don't go below target
    }
  }
  return true;
}

// Function to handle the changing of pwm and direction
bool motorStep(MotorID motorID, float rpm) 
{

  MotorState state = motorStates[motorID];
  Group group = (motorID % 2 == 0) ? LEFT : RIGHT; // Determine group based on motor ID
  
  switch(state) {

    // Motor running normally
    case RUNNING:
    {
      // If we need to change direction, first transition to STOPPING state
      if (pwmValue[motorID] * target_PWM[motorID] < 0) { 
        state = STOPPING;
      } 
      else{ // Otherwise, ramp towards target PWM
        if (pwmValue[motorID] < target_PWM[motorID]) {
          pwmValue[motorID] += RAMP_STEP; // Ramp up speed
          if (pwmValue[motorID] > target_PWM[motorID]) {
            pwmValue[motorID] = target_PWM[motorID]; // Don't exceed target
          }
        } else if (pwmValue[motorID] > target_PWM[motorID]) {
          pwmValue[motorID] -= RAMP_STEP; // Ramp down speed
          if (pwmValue[motorID] < target_PWM[motorID]) {
            pwmValue[motorID] = target_PWM[motorID]; // Don't go below target
          }
        }
      }
      break;
    }

    // Motor needs to stop
    case STOPPING:
    {
      if (pwmValue[motorID] != 0) {
        if (pwmValue[motorID] > 0) { // Ramp down if we're going forward
          pwmValue[motorID] -= RAMP_STEP;
          if (pwmValue[motorID] < 0) pwmValue[motorID] = 0;
        }
        else if (pwmValue[motorID] < 0) { // Ramp "up" if we're going reverse (still ramps to 0)
          pwmValue[motorID] += RAMP_STEP;
          if (pwmValue[motorID] > 0) pwmValue[motorID] = 0;
        }
      } else {
        state = WAITING_FOR_STOP;
        // setBrake(motorID, HIGH); // Engage brakes once we've ramped down to 0
      }
      break;
    }
    
    // Wait and check if the motor has actually stopped
    case WAITING_FOR_STOP:
    {
      if (pwmValue[motorID] != 0) {
        state = STOPPING;
        break;
      }
      // Check if the motor has stopped
      if(fabs(rpm) < STOP_THRESHOLD) {
        if (target_PWM[motorID] == 0) {
          state = NEUTRAL; // If we just needed to stop, go to NEUTRAL
        }
        else {
          state = CHANGING_DIRECTION; // If we need to change direction, go to CHANGING_DIRECTION
        }
      }
      break;
    }
    
    // Change direction after confirming motor has stopped
    case CHANGING_DIRECTION:
    {
      if(group == LEFT) {
        setMotorDirection(motorID, leftDirection);
      }
      else {
        setMotorDirection(motorID, rightDirection);
      }
      state = NEUTRAL; // Transition to NEUTRAL to allow ramping up in new direction
      break;
    }
    case NEUTRAL:
    {
      if (target_PWM[motorID] != 0) {
        state = RUNNING; // If we have a non-zero target, start running
        // setBrake(motorID, LOW); // Disengage brake for this motor when we start running again
      }
      break;
    }

  }

  motorStates[motorID] = state;

  if (state == RUNNING)
    return true;
  else
    return false;
        
}

// Function that is called when a new cmd_vel message is received
void cmd_vel_callback(const void * msgin) {
  digitalWrite(LED, !digitalRead(LED));

  const geometry_msgs__msg__Twist * msg = 
      (const geometry_msgs__msg__Twist *)msgin;

  float v = msg->linear.x;     // m/s (normalized -1 to 1)
  float w = msg->angular.z;    // rad/s

  // Clamp
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;

  float steering_left = 0.0f;
  float steering_right = 0.0f;

  // Straight line case
  if (fabs(w) < 0.001f) {
    steering_left  = 0.0f;
    steering_right = 0.0f;
  } 
  else {
    float R = v / w;

    float R_left  = R - (TRACK_WIDTH / 2.0f);
    float R_right = R + (TRACK_WIDTH / 2.0f);

    steering_left  = atan(WHEELBASE / R_left);
    steering_right = atan(WHEELBASE / R_right);
  }

  // Convert to degrees
  steering_left  *= 180.0f / PI;
  steering_right *= 180.0f / PI;

  // Apply to steering motors
  motors[0].setDeg(steering_left);   // Front Left
  motors[1].setDeg(steering_right);  // Front Right
  motors[2].setDeg(-steering_left);  // Rear Left
  motors[3].setDeg(-steering_right); // Rear Right

  // Now compute wheel speeds
  float left_speed  = v;
  float right_speed = v;

  if (fabs(w) > 0.001f) {
    float R = v / w;
    float R_left  = R - (TRACK_WIDTH / 2.0f);
    float R_right = R + (TRACK_WIDTH / 2.0f);

    left_speed  = w * R_left;
    right_speed = w * R_right;
  }

  // Normalize speeds to PWM
  int left_pwm  = (int)(left_speed  * PWM_MAX_VALUE);
  int right_pwm = (int)(right_speed * PWM_MAX_VALUE);

  // Apply to all driven wheels
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (i % 2 == 0)
      target_PWM[i] = left_pwm;
    else
      target_PWM[i] = right_pwm;
  }

  lastCMDVelTime = millis();
}

// Function that is called every RPM_PUB_INTERVAL milliseconds to publish the current RPM
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  if(timer == NULL) {
    return;
  }

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

  // Publish RPM data
  for (int i = 0; i < MOTOR_COUNT; i++) {
    rpm_msg.data.data[i] = rpm[i];
  }

  rcl_publish(&rpm_pub, &rpm_msg, NULL);



  lastTime = currentTime;
  
  // Check if cmd_vel message has timed out
  currentCMDVelTime = millis();
  if (currentCMDVelTime - lastCMDVelTime > CMD_VEL_TIMEOUT_MS) {
    for(int i = 0; i < MOTOR_COUNT; i++) {
      target_PWM[i] = 0;
      motorStates[i] = STOPPING;
    }
    
  } 

  if (target_PWM[0] > 0) {
    leftDirection = FORWARD;
  }
  else if (target_PWM[0] < 0) {
    leftDirection = REVERSE;
  }
  if( target_PWM[1] > 0) {
    rightDirection = FORWARD;
  }
  else if (target_PWM[1] < 0) {
    rightDirection = REVERSE;
  }

  // Conduct a step. Updates PWM and direction as necessary.
  for(int i = 0; i < MOTOR_COUNT; i++) {
    // SAR ONLY: We don't need to go reverse ;)
    // motorStep(static_cast<MotorID>(i), rpm[i]);
    simpleMotorSetp(static_cast<MotorID>(i), target_PWM[i]);
  }


  // Publish PWM data
  for (int i = 0; i < MOTOR_COUNT; i++) {
    pwm_msg.data.data[i] = pwmValue[i];
  }
  rcl_publish(&pwm_pub, &pwm_msg, NULL);


  for(int i = 0; i < MOTOR_COUNT; i++) {
    setMotorPWM(static_cast<MotorID>(i), pwmValue[i]);
  }

}

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

void setup() {

  Serial.begin(115200);
  while (!Serial && millis() < 5000) {}
  delay(1000);
  set_microros_serial_transports(Serial);




  pinMode(LED, OUTPUT);

  for(int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(MOTOR_PWM_PINS[i], OUTPUT);
    analogWrite(MOTOR_PWM_PINS[i], 0); // Initialize to 0
  }

  // Setup Steering motors
    for(int i = 0; i < MOTOR_COUNT; i++) {
        motors[i] = SteeringMotor(STEERING_MOTOR_PINS[i][0], STEERING_MOTOR_PINS[i][1], STEERING_MOTOR_PINS[i][2], STEERING_MOTOR_PINS[i][3], STEERING_MOTOR_PINS[i][4]);
        motors[i].setup();
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

  // Create subscriber for steering angles
  /* RCCHECK(rclc_subscription_init_default(
      &steering_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      STEERING_ANGLE_TOPIC)); */

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
  int num_handles = 3; // 1 for cmd_vel subscription, 1 for steering subscription, 1 for timer
  RCCHECK(rclc_executor_init(&executor, &support.context, num_handles, &allocator));

  // Add cmd_vel subscription to executor
  RCCHECK(rclc_executor_add_subscription(
      &executor, 
      &cmd_vel_sub, 
      &cmd_vel_msg,
      &cmd_vel_callback, 
      ON_NEW_DATA));  

  /* // Initialze steering message
  steering_msg.data.data = (float*) malloc(sizeof(float) * MOTOR_COUNT);
  steering_msg.data.size = 0;
  steering_msg.data.capacity = MOTOR_COUNT;

  for(int i = 0; i < MOTOR_COUNT; i++)
      steering_msg.data.data[i] = 0.0f;

  // Add steering subscription to executor
  RCCHECK(rclc_executor_add_subscription(
      &executor,
      &steering_sub,
      &steering_msg,
      &steering_callback,
      ON_NEW_DATA)); */
  
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
  lastCMDVelTime = millis();
  lastSteeringCmdTime = millis();

}

void loop() {

  // Spin the executor to handle incoming messages and timer events
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

  // Steering
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