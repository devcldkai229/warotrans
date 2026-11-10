# Battery SOC estimate with the WaroTrans-specific fields sensor_msgs/BatteryState cannot carry.
string MODE_VOLTAGE_ONLY=VOLTAGE_ONLY
string MODE_SIMULATED=SIMULATED
string MODE_UNAVAILABLE=UNAVAILABLE
string CONFIDENCE_MEDIUM=MEDIUM
string CONFIDENCE_LOW=LOW
string CONFIDENCE_NONE=NONE
# Voltage alone cannot tell charging apart: UNKNOWN unless an independent source confirms it.
string CHARGING_UNKNOWN=UNKNOWN
string CHARGING_CHARGING=CHARGING
string CHARGING_NOT_CHARGING=NOT_CHARGING

std_msgs/Header header
string mode
string confidence
string charging_state
# 0..1, NaN when unknown
float32 percentage
# filtered pack voltage, NaN when unknown
float32 voltage
# load-compensated open-circuit voltage estimate, NaN when unknown
float32 ocv_estimate
# true when the robot has been still long enough for the voltage to be near OCV
bool rested
# empty, or why the estimate is unavailable (stale, adc_disconnected, adc_saturated, out_of_range, ...)
string fault
# seconds since the last valid ADC sample, NaN when none was ever received
float32 sample_age_s
