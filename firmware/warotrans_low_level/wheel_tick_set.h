#pragma once

#include <stdint.h>

// Signed (encoder_sign applied, same convention as the telemetry "O" line) snapshot of all four wheels.
// In a header, not in the .ino: the Arduino builder inserts function prototypes near the top of the sketch,
// before any type defined in it, so "WheelTickSet signed_ticks();" failed with "does not name a type".
struct WheelTickSet {
  int32_t front_left;
  int32_t front_right;
  int32_t rear_left;
  int32_t rear_right;
};
