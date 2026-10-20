// Pin scanner: polls every usable input GPIO of a NodeMCU-32S and reports, once per
// second, which pins carry RC-style PWM (high pulses of 800-2200 us) and which sit
// constantly HIGH. Skips GPIO1/3 (USB serial) and GPIO6-11 (SPI flash).
//
// Output line per active pin:
//   GPIO35  RC PWM  pulses=50  width avg=1502 min=1498 max=1506 us
// plus "steady HIGH: ..." for pins that never went low.

#include <Arduino.h>

const int PINS[] = {0, 2, 4, 5, 12, 13, 14, 15, 16, 17, 18, 19, 21, 22, 23,
                    25, 26, 27, 32, 33, 34, 35, 36, 39};
constexpr int N = sizeof(PINS) / sizeof(PINS[0]);

struct Stat {
  bool level;
  uint32_t rise_us;
  uint32_t rc_pulses, other_pulses;
  uint32_t sum_us, min_us, max_us;
  bool went_low, went_high;
};
Stat st[N];

void reset_stats() {
  for (int i = 0; i < N; i++) {
    st[i].rc_pulses = st[i].other_pulses = 0;
    st[i].sum_us = 0;
    st[i].min_us = UINT32_MAX;
    st[i].max_us = 0;
    st[i].went_low = !st[i].level;
    st[i].went_high = st[i].level;
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  for (int i = 0; i < N; i++) {
    pinMode(PINS[i], INPUT);  // no pull-ups: don't disturb whatever is connected
    st[i].level = digitalRead(PINS[i]);
  }
  reset_stats();
  Serial.println("\npin_scan ready: move the steering wheel and trigger");
}

void loop() {
  static uint32_t last_report = millis();

  // Tight polling loop: ~24 pins x ~1 us, plenty for 1-2 ms pulses
  for (int i = 0; i < N; i++) {
    bool v = digitalRead(PINS[i]);
    if (v == st[i].level) continue;
    uint32_t now = micros();
    st[i].level = v;
    if (v) {
      st[i].rise_us = now;
      st[i].went_high = true;
    } else {
      st[i].went_low = true;
      uint32_t w = now - st[i].rise_us;
      if (w >= 800 && w <= 2200) {
        st[i].rc_pulses++;
        st[i].sum_us += w;
        if (w < st[i].min_us) st[i].min_us = w;
        if (w > st[i].max_us) st[i].max_us = w;
      } else {
        st[i].other_pulses++;
      }
    }
  }

  if (millis() - last_report >= 1000) {
    last_report = millis();
    bool any = false;
    String steady_high;
    for (int i = 0; i < N; i++) {
      const Stat &s = st[i];
      if (s.rc_pulses >= 10) {
        Serial.printf("GPIO%-2d  RC PWM  pulses=%u  width avg=%u min=%u max=%u us\n", PINS[i],
                      s.rc_pulses, s.sum_us / s.rc_pulses, s.min_us, s.max_us);
        any = true;
      } else if (s.other_pulses >= 10) {
        Serial.printf("GPIO%-2d  toggling but not RC PWM (%u edges, noise/floating?)\n", PINS[i],
                      s.other_pulses);
      }
      if (s.went_high && !s.went_low) steady_high += " GPIO" + String(PINS[i]);
    }
    if (!any) Serial.println("no RC PWM on any pin");
    if (steady_high.length()) Serial.println("steady HIGH:" + steady_high);
    Serial.println("--");
    reset_stats();
  }
}
