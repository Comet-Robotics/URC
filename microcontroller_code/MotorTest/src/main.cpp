

#include "../setup.cpp"

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

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){digitalWrite(LED,!digitalRead(LED)); delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; }

// Variables for speed calculation
unsigned long lastTime = 0;
long lastPulseCount = 0;
int pwmValue = 0;
int target_PWM = 0;
bool leftDirection = true; // true for forward, false for reverse
bool rightDirection = true; // true for forward, false for reverse

// Micro-ROS variables
rcl_subscription_t cmd_vel_sub;
rcl_publisher_t rpm_pub;
rcl_node_t node;
rcl_allocator_t allocator;
rcl_timer_t timer;

rclc_support_t support;
rclc_executor_t executor;


// Micro-ROS messages
geometry_msgs__msg__Twist cmd_vel_msg; // Equates to geometry_msgs::msg::Twist
std_msgs__msg__Float32MultiArray rpm_msg; // Equates to std_msgs::msg::Float32MultiArray

const unsigned long RPM_PUB_INTERVAL = 100; // Publish RPM every 100ms

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

  // Calculate motor RPM
  unsigned long currentTime = millis();
  long currentPulseCount = readMotorPulses(FRONT_LEFT_MOTOR);

  long pulseDiff = currentPulseCount - lastPulseCount;
  float timeInterval = (currentTime - lastTime) / 1000.0;

  float pulsesPerSecond = pulseDiff / timeInterval;
  float rpm = (pulsesPerSecond * 2.0 * 60.0) / PULSES_PER_REVOLUTION;

  // Publish RPM data
  rpm_msg.data.data[0] = rpm;
  rcl_publish(&rpm_pub, &rpm_msg, NULL);

  lastPulseCount = currentPulseCount;
  lastTime = currentTime;
  
  // Gradually adjust PWM towards target_PWM
  if (pwmValue < target_PWM) {
    pwmValue += 1; // Ramp up speed
    if (pwmValue > target_PWM) {
      pwmValue = target_PWM; // Don't exceed target
    }
  } else if (pwmValue > target_PWM) {
    pwmValue -= 1; // Ramp down speed
    if (pwmValue < target_PWM) {
      pwmValue = target_PWM; // Don't go below target
    }
  }

  // if(pwmValue < 0) pwmValue = 0; // Ensure PWM doesn't go negative

  // // Handle direction changes
  // if(target_PWM < 0 && pwmValue == 0) {
  //   target_PWM = -target_PWM; // Make target positive for reversing

  //   // Toggle direction
  //   leftDirection = !leftDirection; 
  //   rightDirection = !rightDirection; 
  // }

  // Set PWM for all motors
  // TODO: Adjust this so all motors turn in the same direction
  for(int i = 0; i < MOTOR_COUNT; i++) {
    setMotorPWM(static_cast<MotorID>(i), pwmValue);
  }
}


void setup() {
  /*
  Serial.begin(115200);  
  
  // Initialize the motor speed reader
  initMotorSpeedReader();
  
  pinMode(SLOWDOWN, INPUT_PULLUP);  // Changed to GPIO 24 for Teensy
  pinMode(LED, OUTPUT);
  // Serial.println("Brushless Motor ESC Speed Reader Initialized (Teensy  )");
  // Serial.println("Connect ESC speed signal to GPIO 2");
  
  lastTime = millis();
  digitalWrite(LED, LOW); // Turn on built-in LED 
  */


  //set_microros_transports();
  Serial.begin(115200);
  delay(2000); // Wait for serial to initialize
  set_microros_serial_transports(Serial);
  delay(2000);



  pinMode(LED, OUTPUT);

  initMotorSpeedReader();

  // Initialize allocator
  allocator = rcl_get_default_allocator();
  executor = rclc_executor_get_zero_initialized_executor();

  if(rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Error initializing rclc support");
  }
  // RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  // RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));
  if(rclc_node_init_default(&node, "motor_node", "", &support) != RCL_RET_OK) {
    Serial.println("Error initializing node");
  }

  // Create cmd_vel subscriber
  // RCCHECK(rclc_subscription_init_default(
  //     &cmd_vel_sub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
  //     "/cmd_vel"));
  if(rclc_subscription_init_default(
      &cmd_vel_sub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      "/cmd_vel") != RCL_RET_OK) {
    Serial.println("Error initializing cmd_vel subscriber");
  }


  // Create rpm publisher
  // RCCHECK(rclc_publisher_init_default(
  //     &rpm_pub,
  //     &node,
  //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
  //     "/rpm"));
  if(rclc_publisher_init_default(
      &rpm_pub,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      "/rpm") != RCL_RET_OK) {
    Serial.println("Error initializing rpm publisher");
  }
  
  // Create timer for publishing RPM
  RCCHECK(rclc_timer_init_default(
      &timer,
      &support,
      RCL_MS_TO_NS(RPM_PUB_INTERVAL),
      timer_callback));
  // if(rclc_timer_init_default(
  //     &timer,
  //     &support,
  //     RCL_MS_TO_NS(RPM_PUB_INTERVAL),
  //     timer_callback) != RCL_RET_OK) {
  //   Serial.println("Error initializing timer");
  // }
  
  // Create executor
  // RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  if(rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("Error initializing executor");
  }

  // Add subscription to executor
  // RCCHECK(rclc_executor_add_subscription(
  //     &executor, 
  //     &cmd_vel_sub, 
  //     &cmd_vel_msg,
  //     &cmd_vel_callback, 
  //     ON_NEW_DATA));
  if(rclc_executor_add_subscription(
      &executor, 
      &cmd_vel_sub, 
      &cmd_vel_msg,
      &cmd_vel_callback, 
      ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Error adding subscription to executor");
  }
  
  
  // Add timer to executor
  RCCHECK(rclc_executor_add_timer(
      &executor, 
      &timer));
  // if(rclc_executor_add_timer(
  //     &executor, 
  //     &timer) != RCL_RET_OK) {
  //   Serial.println("Error adding timer to executor");
  // }

  // Initialize RPM message
  rpm_msg.data.capacity = MOTOR_COUNT;
  rpm_msg.data.size = MOTOR_COUNT;
  rpm_msg.data.data = (float *)malloc(MOTOR_COUNT * sizeof(float));  

  
  lastTime = millis();

}

void loop() {

  /* 
  // Calculate and display motor speed every 500ms
  unsigned long currentTime = millis();
  
  digitalWrite(LED, !digitalRead(LED)); // Toggle built-in LED for visual feedback

  if (currentTime - lastTime >= 500) {
    long currentPulseCount = readMotorPulses(FRONT_LEFT_MOTOR);
    long pulseDiff = currentPulseCount - lastPulseCount;
    float timeInterval = (currentTime - lastTime) / 1000.0; // Convert to seconds
    

    digitalWrite(LED, !digitalRead(LED)); // Toggle built-in LED for visual feedback


    // Calculate pulses per second (Hz)


    float pulsesPerSecond = pulseDiff / timeInterval;
    
    // Note: Using RISING interrupt, so multiply by 2 for actual pulse rate
    float rpm = (pulsesPerSecond * 2.0 * 60.0) / PULSES_PER_REVOLUTION;

    bool slowDown = !digitalRead(SLOWDOWN);  // Changed to GPIO 24 for Teensy
    
    // DEBUG: Read encoder pin state directly
    // int encoderPinState = digitalRead(MOTOR_ESC_SPEED_PINS[0]);
    
    // Serial.print("Total Pulses: ");
    // Serial.print(currentPulseCount);
    // Serial.print(" | Pulses/sec: ");
    // Serial.print(pulsesPerSecond);
    // Serial.print(" Hz | Estimated RPM: ");
    // Serial.print(rpm);
    // Serial.print(" | Total rotations: ");
    // Serial.print(currentPulseCount / PULSES_PER_REVOLUTION);
    for (int i = 0; i < MOTOR_COUNT; i++) {
      int encoderPinState = readMotorPulses(static_cast<MotorID>(i));
      Serial.print(" | Encoder pin ");
      Serial.print(i + 1);
      Serial.print(" state: ");
      Serial.println(encoderPinState);
    }
    
    lastPulseCount = currentPulseCount;
    lastTime = currentTime;

    //setMotorPWM(FRONT_LEFT_MOTOR, pwmValue);
    for(int i = 0; i < MOTOR_COUNT; i++) {
      setMotorPWM(static_cast<MotorID>(i), pwmValue);
    }

    if(pwmValue < PWM_MAX_VALUE && !slowDown) {
      pwmValue+=3;
      Serial.print(" Speeding Up ");
      digitalWrite(LED, HIGH);
    }
    else if(pwmValue > 0 && slowDown) {
      pwmValue-=3;
      Serial.print(" Slowing Down ");
      digitalWrite(LED, LOW);
    }
    Serial.println(" | PWM Value Set To: " + String(pwmValue));
  }
  
  // Your motor control code can go here 
  
  */


  // Spin the executor to handle incoming messages and timer events
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
  
}