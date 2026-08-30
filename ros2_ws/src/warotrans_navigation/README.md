# warotrans_navigation

Owns Nav2 configuration and navigation assets.

## Prerequisites

- validated map
- stable `/scan`
- stable `/odom`
- valid TF tree
- localization
- reliable `/cmd_vel` path to ESP32
- safe stop/watchdog
- sufficiently stable closed-loop base control

Nav2 should know ROS-level motion contracts, not MDD10A GPIO/PWM details.
