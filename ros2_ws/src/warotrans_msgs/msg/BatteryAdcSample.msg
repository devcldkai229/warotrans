# One averaged reading of the battery voltage divider (0-25 V module) on the ESP32-S3 ADC pin.
# Raw: conversion to pack volts / SOC is done by warotrans_hardware battery_monitor_node.
std_msgs/Header header
uint32 mcu_millis
# eFuse-calibrated millivolts at the ESP32 pin (analogReadMilliVolts), averaged over sample_count reads
float32 pin_millivolts
uint16 sample_count
