#include "rc_input.h"

namespace rc_input {
namespace {

int pin_of[MAX_CHANNELS];
int channels = 0;
volatile uint32_t rise_us[MAX_CHANNELS];
volatile uint32_t width_us[MAX_CHANNELS];
volatile uint32_t last_us[MAX_CHANNELS];

void IRAM_ATTR handle(int ch) {
  uint32_t now = micros();
  if (digitalRead(pin_of[ch])) {
    rise_us[ch] = now;
  } else {
    uint32_t w = now - rise_us[ch];
    if (w >= MIN_US && w <= MAX_US) {
      width_us[ch] = w;
      last_us[ch] = now;
    }
  }
}

void IRAM_ATTR isr0() { handle(0); }
void IRAM_ATTR isr1() { handle(1); }
void IRAM_ATTR isr2() { handle(2); }
void IRAM_ATTR isr3() { handle(3); }
void (*const ISRS[MAX_CHANNELS])() = {isr0, isr1, isr2, isr3};

}  // namespace

void begin(const int *pins, int n) {
  channels = min(n, MAX_CHANNELS);
  for (int ch = 0; ch < channels; ch++) {
    pin_of[ch] = pins[ch];
    pinMode(pins[ch], INPUT);  // the receiver drives the line; input-only pins are fine
    attachInterrupt(digitalPinToInterrupt(pins[ch]), ISRS[ch], CHANGE);
  }
}

int32_t read(int ch) {
  if (ch < 0 || ch >= channels) return 0;
  noInterrupts();
  uint32_t w = width_us[ch], last = last_us[ch];
  interrupts();
  return (last != 0 && micros() - last < TIMEOUT_US) ? (int32_t)w : 0;
}

uint32_t last_pulse_us(int ch) {
  return (ch >= 0 && ch < channels) ? last_us[ch] : 0;
}

}  // namespace rc_input
