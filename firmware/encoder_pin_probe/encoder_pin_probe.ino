// Encoder wiring probe (2026-11-20). Flash, open the Serial Monitor at 115200, turn ONE wheel slowly by hand.
// Every 200 ms it prints, for each encoder pin, its level (0/1) and how many edges it has seen since boot:
//   FL C2:8 C1:9 | FR C2:10 C1:11 | RL C2:12 C1:13 | RR C2:14 C1:15
// A healthy channel: the level toggles and its edge count rises while its wheel turns (about 1240 edges per
// wheel turn on each pin: ~2480 quadrature ticks per turn, shared by C1 and C2). A pin stuck at 1 with 0 edges: no signal (encoder unpowered, wire on the wrong
// header pin, or loose). The rear pins use the same pull-ups as warotrans_low_level.
// Re-flash warotrans_low_level afterwards: this sketch never drives the motors.
#include <Arduino.h>

struct Probe {
  const char* name;
  uint8_t pin;
  bool pullup;
};

static const Probe probes[] = {
    {"FL_C2", 8, false},  {"FL_C1", 9, false},  {"FR_C2", 10, false}, {"FR_C1", 11, false},
    {"RL_C2", 12, true},  {"RL_C1", 13, true},  {"RR_C2", 14, true},  {"RR_C1", 15, true},
};
static constexpr size_t probe_count = sizeof(probes) / sizeof(probes[0]);
static volatile uint32_t edges[probe_count] = {0};

static void IRAM_ATTR on_edge(void* arg) {
  const uintptr_t i = reinterpret_cast<uintptr_t>(arg);
  edges[i] = edges[i] + 1;
}

void setup() {
  // Motor driver pins low, as in warotrans_low_level (MDD10A PWM 4/6, DIR 5/7): the motors stay off.
  for (uint8_t pin : {4, 5, 6, 7}) {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
  }
  Serial.begin(115200);
  delay(300);
  for (size_t i = 0; i < probe_count; ++i) {
    pinMode(probes[i].pin, probes[i].pullup ? INPUT_PULLUP : INPUT);
    attachInterruptArg(digitalPinToInterrupt(probes[i].pin), on_edge, reinterpret_cast<void*>(i), CHANGE);
  }
  Serial.println("encoder_pin_probe: turn one wheel slowly; name=level/edges");
}

void loop() {
  char line[200];
  size_t n = 0;
  for (size_t i = 0; i < probe_count; ++i) {
    n += snprintf(line + n, sizeof(line) - n, "%s=%d/%lu%s", probes[i].name, digitalRead(probes[i].pin),
                  static_cast<unsigned long>(edges[i]), (i % 2 == 1 && i + 1 < probe_count) ? " | " : " ");
  }
  Serial.println(line);
  delay(200);
}
