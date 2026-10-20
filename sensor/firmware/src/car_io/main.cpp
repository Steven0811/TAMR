// NodeMCU-32S I/O board -> micro-ROS over USB serial.
//   * SHARP GP2Y0A21YK0F (10-80 cm IR distance sensor) on GPIO36
//   * HotRC DS600 receiver PWM: CH1 GPIO34, CH2 GPIO35, CH3 GPIO32
//
// Topics:
//   /range_finder/range    sensor_msgs/Range        (m, frame ir_range_link)    20 Hz
//   /range_finder/voltage  std_msgs/Float32         (V, raw sensor output)      20 Hz
//   /rc/pulses             std_msgs/Int32MultiArray [ch1, ch2, ch3] pulse width in us,
//                                                   0 = no signal on that channel  50 Hz
//
// Serial is owned by the micro-ROS transport: do not Serial.print() anything.
// The on-board LED (GPIO2) shows state: blinking = waiting for agent, on = connected.

#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <sensor_msgs/msg/range.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <math.h>

#include "rc_input.h"

constexpr int SENSOR_PIN = 36;      // ADC1_CH0 (input only)
constexpr int LED_PIN = 2;
constexpr int RANGE_PERIOD_MS = 50;  // 20 Hz
constexpr int RC_PERIOD_MS = 20;     // 50 Hz, same as the receiver frame rate
constexpr int SAMPLES = 15;          // median filter window per reading

constexpr float MIN_RANGE_M = 0.10f;
constexpr float MAX_RANGE_M = 0.80f;

// RC receiver inputs (input-only pins are fine: the receiver drives the line)
constexpr int RC_CHANNELS = 3;
constexpr int RC_PINS[RC_CHANNELS] = {34, 35, 32};

static char FRAME_ID[] = "ir_range_link";

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_publisher_t range_pub;
rcl_publisher_t voltage_pub;
rcl_publisher_t rc_pub;
rcl_timer_t range_timer;
rcl_timer_t rc_timer;
rclc_executor_t executor;

sensor_msgs__msg__Range range_msg;
std_msgs__msg__Float32 voltage_msg;
std_msgs__msg__Int32MultiArray rc_msg;
int32_t rc_buffer[RC_CHANNELS];

enum class AgentState { WAITING, AVAILABLE, CONNECTED, DISCONNECTED };
AgentState state = AgentState::WAITING;

#define RCCHECK(fn) { if ((fn) != RCL_RET_OK) return false; }
#define EXECUTE_EVERY_N_MS(ms, X) do { \
    static int64_t t_ = -1; \
    if (t_ == -1) t_ = uxr_millis(); \
    if ((int32_t)(uxr_millis() - t_) > (ms)) { X; t_ = uxr_millis(); } \
  } while (0)

// ------------------------------------------------------------------ range finder
// Median of SAMPLES readings, in volts. The sensor output has spikes at each
// internal measurement cycle (~38 ms), which a median rejects well.
float read_voltage() {
  int mv[SAMPLES];
  for (int i = 0; i < SAMPLES; i++) {
    mv[i] = analogReadMilliVolts(SENSOR_PIN);  // eFuse-calibrated
    delayMicroseconds(200);
  }
  // insertion sort (small N)
  for (int i = 1; i < SAMPLES; i++) {
    int v = mv[i], j = i - 1;
    while (j >= 0 && mv[j] > v) { mv[j + 1] = mv[j]; j--; }
    mv[j + 1] = v;
  }
  return mv[SAMPLES / 2] / 1000.0f;
}

// Power-law fit of the datasheet curve: d[cm] = 27.728 * V^-1.2045.
// Below 10 cm the output folds back, so readings there are ambiguous.
float voltage_to_range_m(float v) {
  if (v < 0.05f) return INFINITY;  // nothing in front / sensor disconnected
  float d = 27.728f * powf(v, -1.2045f) / 100.0f;
  if (d > MAX_RANGE_M) return INFINITY;    // REP 117: beyond max -> +Inf
  if (d < MIN_RANGE_M) return -INFINITY;   // too close -> -Inf
  return d;
}

// ------------------------------------------------------------------ callbacks
void range_timer_callback(rcl_timer_t *t, int64_t /*last_call_time*/) {
  if (t == NULL) return;

  float v = read_voltage();
  int64_t ns = rmw_uros_epoch_nanos();  // agent-synchronised time

  range_msg.header.stamp.sec = (int32_t)(ns / 1000000000LL);
  range_msg.header.stamp.nanosec = (uint32_t)(ns % 1000000000LL);
  range_msg.range = voltage_to_range_m(v);
  voltage_msg.data = v;

  rcl_publish(&range_pub, &range_msg, NULL);
  rcl_publish(&voltage_pub, &voltage_msg, NULL);
}

void rc_timer_callback(rcl_timer_t *t, int64_t /*last_call_time*/) {
  if (t == NULL) return;
  for (int i = 0; i < RC_CHANNELS; i++) rc_buffer[i] = rc_input::read(i);
  rcl_publish(&rc_pub, &rc_msg, NULL);
}

// ------------------------------------------------------------------ micro-ROS entities
bool create_entities() {
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "esp32_io", "", &support));
  RCCHECK(rclc_publisher_init_best_effort(
      &range_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range), "range_finder/range"));
  RCCHECK(rclc_publisher_init_best_effort(
      &voltage_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "range_finder/voltage"));
  RCCHECK(rclc_publisher_init_best_effort(
      &rc_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "rc/pulses"));
  RCCHECK(rclc_timer_init_default(&range_timer, &support, RCL_MS_TO_NS(RANGE_PERIOD_MS), range_timer_callback));
  RCCHECK(rclc_timer_init_default(&rc_timer, &support, RCL_MS_TO_NS(RC_PERIOD_MS), rc_timer_callback));
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &range_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &rc_timer));

  rmw_uros_sync_session(1000);
  return true;
}

void destroy_entities() {
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  rcl_publisher_fini(&range_pub, &node);
  rcl_publisher_fini(&voltage_pub, &node);
  rcl_publisher_fini(&rc_pub, &node);
  rcl_timer_fini(&range_timer);
  rcl_timer_fini(&rc_timer);
  rclc_executor_fini(&executor);
  rcl_node_fini(&node);
  rclc_support_fini(&support);
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  analogReadResolution(12);
  analogSetPinAttenuation(SENSOR_PIN, ADC_11db);  // 0-~3.1 V input range
  rc_input::begin(RC_PINS, RC_CHANNELS);

  Serial.begin(115200);
  set_microros_serial_transports(Serial);

  range_msg.header.frame_id.data = FRAME_ID;
  range_msg.header.frame_id.size = strlen(FRAME_ID);
  range_msg.header.frame_id.capacity = sizeof(FRAME_ID);
  range_msg.radiation_type = sensor_msgs__msg__Range__INFRARED;
  range_msg.field_of_view = 0.1f;  // rad, narrow IR beam (approximate)
  range_msg.min_range = MIN_RANGE_M;
  range_msg.max_range = MAX_RANGE_M;

  // Static storage for the RC array (no dynamic allocation on the MCU)
  rc_msg.data.data = rc_buffer;
  rc_msg.data.size = RC_CHANNELS;
  rc_msg.data.capacity = RC_CHANNELS;
  rc_msg.layout.dim.data = NULL;
  rc_msg.layout.dim.size = 0;
  rc_msg.layout.dim.capacity = 0;
  rc_msg.layout.data_offset = 0;
}

void loop() {
  switch (state) {
    case AgentState::WAITING:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1))
                                          ? AgentState::AVAILABLE : AgentState::WAITING;);
      digitalWrite(LED_PIN, (millis() / 250) % 2);  // blink
      break;
    case AgentState::AVAILABLE:
      state = create_entities() ? AgentState::CONNECTED : AgentState::WAITING;
      if (state == AgentState::WAITING) destroy_entities();
      break;
    case AgentState::CONNECTED:
      EXECUTE_EVERY_N_MS(1000, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 3))
                                           ? AgentState::CONNECTED : AgentState::DISCONNECTED;);
      if (state == AgentState::CONNECTED) {
        digitalWrite(LED_PIN, HIGH);
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AgentState::DISCONNECTED:
      destroy_entities();
      state = AgentState::WAITING;
      break;
  }
}
