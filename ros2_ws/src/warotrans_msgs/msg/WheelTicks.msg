int64 left_ticks
int64 right_ticks
uint32 mcu_millis
# Rear wheel encoders (firmware 2026-11-20, "O" line with 4 wheels). rear_valid false = front-only firmware,
# rear_*_ticks are then 0. left/right above are the FRONT wheels, as before.
int64 rear_left_ticks
int64 rear_right_ticks
bool rear_valid
