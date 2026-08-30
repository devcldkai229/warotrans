"""Placeholder launch file for WaroTrans sensor drivers.

Future implementation should start the LiDAR driver only, using the measured
frame name from warotrans_description and the topic contract in interfaces.md.

This file intentionally launches nothing until the owning subsystem has
passed its previous engineering checkpoint.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
