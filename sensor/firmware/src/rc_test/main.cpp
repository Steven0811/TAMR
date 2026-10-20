// Diagnostic: read the RC receiver with the same rc_input library as car_io and print
// the pulse widths over Serial (no micro-ROS), 5 times a second.
//
//   ch1=1492 ch2=1478 ch3=0   (last pulse: 1205512 1185498 0 us, now 1205531)
//
// 0 means no valid pulse on that channel for 100 ms. A channel that mirrors another
// one exactly is usually an unconnected pin picking up crosstalk.

#include <Arduino.h>

#include "rc_input.h"

constexpr int RC_CHANNELS = 3;
constexpr int RC_PINS[RC_CHANNELS] = {34, 35, 32};  // same wiring as car_io

void setup() {
  Serial.begin(115200);
  rc_input::begin(RC_PINS, RC_CHANNELS);
  Serial.println("\nrc_test ready");
}

void loop() {
  Serial.printf("ch1=%d ch2=%d ch3=%d   (last pulse: %u %u %u us, now %u)\n",
                rc_input::read(0), rc_input::read(1), rc_input::read(2),
                (unsigned)rc_input::last_pulse_us(0), (unsigned)rc_input::last_pulse_us(1),
                (unsigned)rc_input::last_pulse_us(2), (unsigned)micros());
  delay(200);
}
