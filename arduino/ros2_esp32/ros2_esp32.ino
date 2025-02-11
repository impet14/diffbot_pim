/*
 * Optimized ESP32 micro-ROS Differential Drive Robot Controller with PID
 * ROS2 Humble - USB/Serial Communication
 * micro-ROS Client for cmd_vel Subscription and Diagnostic Monitoring
 */

#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

// ===== Pin and Constant Definitions =====
#define WHEEL_DISTANCE 0.265  // Distance between wheels [m]
#define ENCODER_A_LEFT 27
#define ENCODER_B_LEFT 14
#define ENCODER_A_RIGHT 25
#define ENCODER_B_RIGHT 26
#define MOTOR_LEFT_IN1 16
#define MOTOR_LEFT_IN2 4
#define MOTOR_RIGHT_IN1 2
#define MOTOR_RIGHT_IN2 15
#define MAX_SPEED 255       // Maximum PWM value
#define PID_INTERVAL_MS 50  // PID control interval in milliseconds

// ----- PID Constants -----
const double Kp = 13.0;
const double Ki = 0.001;
const double Kd = 0.001;
const double Ka = 1.5; //gain for robot turning

// Conversion factor from m/s to encoder ticks per PID interval
// (Example: based on a 214mm wheel circumference, 960 ticks per revolution,
//  and PID interval in ms)
const double mps_ticks = 1.0 / ((214 * 1e-3 / 960) / (PID_INTERVAL_MS * 1e-3));

// ===== Global Variables =====

// ROS2 objects
rcl_subscription_t cmd_vel_subscriber;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
geometry_msgs__msg__Twist cmd_vel_msg;

// Diagnostic monitoring publisher (optional)
rcl_publisher_t monitor_publisher;
geometry_msgs__msg__Twist monitor_msg;

// Encoder tick counters (updated in ISRs)
volatile long left_encoder_ticks = 0;
volatile long right_encoder_ticks = 0;

// PID setpoints (in encoder ticks per PID interval)
// Modify these default speeds (in m/s) as needed.
#define LSPEED 0.0
#define RSPEED 0.0
double left_speed_setpoint = LSPEED * mps_ticks;
double right_speed_setpoint = RSPEED * mps_ticks;

// PID outputs (for motor PWM)
double left_motor_output = 0;
double right_motor_output = 0;

// Measured speeds (in encoder ticks per PID interval)
volatile double left_speed_actual = 0;
volatile double right_speed_actual = 0;
unsigned long last_time = 0;

// PID control internal variables
double integral_left = 0, integral_right = 0;
double prev_error_left = 0, prev_error_right = 0;

// Variables to store last received cmd_vel (for diagnostics)
float linearVel = 0.0;
float angularVel = 0.0;

// ===== Function Prototypes =====
void cmd_vel_callback(const void *msgin);
void pidControlLoop();
void leftEncoderISR();
void rightEncoderISR();
void setLeftMotorSpeed(int speed);
void setRightMotorSpeed(int speed);

void setup() {
  // Initialize micro-ROS transports
  set_microros_transports();

  // Initialize motor control pins
  pinMode(MOTOR_LEFT_IN1, OUTPUT);
  pinMode(MOTOR_LEFT_IN2, OUTPUT);
  pinMode(MOTOR_RIGHT_IN1, OUTPUT);
  pinMode(MOTOR_RIGHT_IN2, OUTPUT);

  // Initialize encoder pins with pull-ups for better signal stability
  pinMode(ENCODER_A_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_B_LEFT, INPUT_PULLUP);
  pinMode(ENCODER_A_RIGHT, INPUT_PULLUP);
  pinMode(ENCODER_B_RIGHT, INPUT_PULLUP);

  // Attach interrupts for encoders
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_LEFT), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(ENCODER_A_RIGHT), rightEncoderISR, RISING);

  // Initialize micro-ROS node and executor
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_diff_drive", "", &support);

  // Initialize the cmd_vel subscriber
  rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel");

  // Initialize the diagnostic publisher (for monitoring cmd_vel data)
  rclc_publisher_init_default(
    &monitor_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel_monitor");

  // Initialize the executor and add the subscriber callback
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);

  last_time = millis();
}

void loop() {
  unsigned long current_time = millis();
  if (current_time - last_time >= PID_INTERVAL_MS) {
    // Atomically read and reset encoder counts
    noInterrupts();
    left_speed_actual  = left_encoder_ticks;
    right_speed_actual = right_encoder_ticks;
    left_encoder_ticks = 0;
    right_encoder_ticks = 0;
    interrupts();

    pidControlLoop();
    last_time = current_time;
  }
  // Process incoming ROS2 messages (non-blocking)
  rclc_executor_spin_some(&executor, 0);
  delay(5);  // Small delay to yield CPU time
}

// ===== Callback and Control Functions =====

// Callback for receiving cmd_vel commands from ROS2
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  linearVel = msg->linear.x;
  angularVel = msg->angular.z;
  
  angularVel = angularVel* Ka; // gain for adjust angular
  // Compute new setpoints (in encoder ticks per PID interval)
  left_speed_setpoint  = (linearVel - (WHEEL_DISTANCE * angularVel) / 2) * mps_ticks;
  right_speed_setpoint = (linearVel + (WHEEL_DISTANCE * angularVel) / 2) * mps_ticks;

  // Publish diagnostic data: the received linear and angular velocities.
  monitor_msg.linear.x  = linearVel;
  monitor_msg.angular.z = angularVel;
  rcl_publish(&monitor_publisher, &monitor_msg, NULL);
}

// PID control loop implementation
void pidControlLoop() {
  double error_left  = left_speed_setpoint  - left_speed_actual;
  double error_right = right_speed_setpoint - right_speed_actual;

  // Update integral terms
  integral_left  += error_left;
  integral_right += error_right;

  // Compute PID outputs (without explicit dt scaling)
  double control_output_left  = Kp * error_left  + Ki * integral_left  + Kd * (error_left  - prev_error_left);
  double control_output_right = Kp * error_right + Ki * integral_right + Kd * (error_right - prev_error_right);

  // Drive motors with the computed PWM outputs
  setLeftMotorSpeed(control_output_left);
  setRightMotorSpeed(control_output_right);

  // Update previous error values
  prev_error_left  = error_left;
  prev_error_right = error_right;
}

// ===== Interrupt Service Routines =====

// Left encoder ISR: determines rotation direction based on second channel state.
void leftEncoderISR() {
  if (digitalRead(ENCODER_B_LEFT) == HIGH) {
    left_encoder_ticks++;
  } else {
    left_encoder_ticks--;
  }
}

// Right encoder ISR: determines rotation direction based on second channel state.
void rightEncoderISR() {
  if (digitalRead(ENCODER_B_RIGHT) == LOW) {
    right_encoder_ticks++;
  } else {
    right_encoder_ticks--;
  }
}

// ===== Motor Control Functions =====

void setLeftMotorSpeed(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed > 0) {
    analogWrite(MOTOR_LEFT_IN1, speed);
    analogWrite(MOTOR_LEFT_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOTOR_LEFT_IN1, 0);
    analogWrite(MOTOR_LEFT_IN2, -speed);
  } else {
    analogWrite(MOTOR_LEFT_IN1, 0);
    analogWrite(MOTOR_LEFT_IN2, 0);
  }
}

void setRightMotorSpeed(int speed) {
  speed = constrain(speed, -MAX_SPEED, MAX_SPEED);
  if (speed > 0) {
    analogWrite(MOTOR_RIGHT_IN1, speed);
    analogWrite(MOTOR_RIGHT_IN2, 0);
  } else if (speed < 0) {
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, -speed);
  } else {
    analogWrite(MOTOR_RIGHT_IN1, 0);
    analogWrite(MOTOR_RIGHT_IN2, 0);
  }
}
