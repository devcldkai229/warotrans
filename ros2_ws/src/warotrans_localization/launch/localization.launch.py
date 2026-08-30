"""Placeholder launch file for WaroTrans localization.

Future implementation should start the selected EKF/localization stack only after odometry and map validation.

This file intentionally launches nothing until the owning subsystem has
passed its previous engineering checkpoint.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
