"""Placeholder launch file for WaroTrans fleet bridge.

Future implementation should bridge high-level tasks/status only. It must not command PWM directly.

This file intentionally launches nothing until the owning subsystem has
passed its previous engineering checkpoint.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
