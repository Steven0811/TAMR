// RC receiver PWM input (HotRC F-06A or any standard receiver).
//
// Measures the high-pulse width of each channel with a CHANGE interrupt.
// Pulses outside 800-2200 us are ignored; a channel that has produced no valid
// pulse for 100 ms reads as 0 ("no signal").
#pragma once

#include <Arduino.h>

namespace rc_input {

constexpr int MAX_CHANNELS = 4;
constexpr uint32_t MIN_US = 800, MAX_US = 2200;  // plausible pulse widths
constexpr uint32_t TIMEOUT_US = 100000;           // no pulse for 100 ms -> signal lost

// Attach interrupts on `n` pins (n <= MAX_CHANNELS), channel i = pins[i].
void begin(const int *pins, int n);

// Latest pulse width of a channel in microseconds, 0 if the channel has gone quiet.
int32_t read(int ch);

// Time (micros()) of the last valid pulse on a channel, 0 if none yet. For diagnostics.
uint32_t last_pulse_us(int ch);

}  // namespace rc_input
