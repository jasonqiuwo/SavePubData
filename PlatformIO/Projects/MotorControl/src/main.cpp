#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int64.h>
#include <std_msgs/msg/u_int32.h>

#include <stdio.h>
#include <unistd.h>

// Pins and variables for the encoder
int E1 = 5;
int M1 = 6;

const int encoderAPin = 7;
const int encoderBPin = 8;

volatile long encoderValue = 0;
volatile bool lastAState = LOW;
volatile bool lastBState = LOW;
int degree = 0;
float PPR = 3590.4;

// ROS 2 variables
rcl_publisher_t degree_publisher;
rcl_publisher_t timestamp_publisher;

std_msgs__msg__Int32 degree_msg;
std_msgs__msg__UInt32 timestamp_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t degree_timer;
rcl_timer_t timestamp_timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

// Error handling loop
void error_loop() {
  while(1) {
    delay(100);
  }
}

// Timer callback for publishing the degree value
void encoder_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    int rev = round(encoderValue / PPR);
    int position = round(encoderValue - rev * PPR);
    degree = (position * 360) / PPR;
    if (degree < 0) {
      degree += 360;
    }
    degree_msg.data = degree;
    RCSOFTCHECK(rcl_publish(&degree_publisher, &degree_msg, NULL));
  }
}

// Timer callback for publishing the timestamp
void timestamp_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
	(void) timer;
  RCSOFTCHECK(rmw_uros_sync_session(1000));

  int64_t time = rmw_uros_epoch_millis();
  timestamp_msg.data = time;
  RCSOFTCHECK(rcl_publish(&timestamp_publisher, &timestamp_msg, NULL));
}

// ISR for encoder A
void encoderAISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (aState != lastAState) {
    if (aState == bState) {
      encoderValue++;
    } else {
      encoderValue--;
    }
  }
  
  lastAState = aState;
}

// ISR for encoder B
void encoderBISR() {
  bool aState = digitalRead(encoderAPin);
  bool bState = digitalRead(encoderBPin);
  
  if (bState != lastBState) {
    if (aState == bState) {
      encoderValue--;
    } else {
      encoderValue++;
    }
  }
  
  lastBState = bState;
}

void setup() {
  // Configure serial transport
  Serial.begin(115200);
  set_microros_serial_transports(Serial);
  delay(2000);

  // Initialize pins
  pinMode(M1, OUTPUT);
  pinMode(E1, OUTPUT);
  pinMode(encoderAPin, INPUT);
  pinMode(encoderBPin, INPUT);
  
  digitalWrite(M1, HIGH);
  analogWrite(E1, 100);

  // Attach interrupts for encoder
  attachInterrupt(digitalPinToInterrupt(encoderAPin), encoderAISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderBPin), encoderBISR, CHANGE);

  // ROS 2 setup
  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_platformio_node", "", &support));

  // Create degree publisher
  RCCHECK(rclc_publisher_init_default(
    &degree_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "degree"));

  // Create timestamp publisher
  RCCHECK(rclc_publisher_init_default(
    &timestamp_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "timestamp"));

  // Create degree timer
  const unsigned int degree_timer_timeout = 1;  // Adjust the timer period as needed
  RCCHECK(rclc_timer_init_default(
    &degree_timer,
    &support,
    RCL_MS_TO_NS(degree_timer_timeout),
    encoder_callback));

  // Create timestamp timer
  const unsigned int timestamp_timer_timeout = 1;  // Adjust the timer period as needed
  RCCHECK(rclc_timer_init_default(
    &timestamp_timer,
    &support,
    RCL_MS_TO_NS(timestamp_timer_timeout),
    timestamp_callback));

  // Create executor and add both timers
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &degree_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &timestamp_timer));

  // Initialize message data
  degree_msg.data = 0;
  timestamp_msg.data = 0;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
