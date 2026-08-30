"""Placeholder launch file for WaroTrans Nav2.

Future implementation should launch Nav2 only after mapping, localization, TF, odometry, and motor-control gates pass.

This file intentionally launches nothing until the owning subsystem has
passed its previous engineering checkpoint.
"""

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription([])
