import math

from warotrans_fleet.transforms import quaternion_to_yaw


def test_yaw_identity_is_zero():
    assert abs(quaternion_to_yaw(0.0, 0.0, 0.0, 1.0)) < 1e-9


def test_yaw_ninety_degrees():
    # Quaternion for +90 deg about Z: z=sin(45deg), w=cos(45deg)
    z = math.sin(math.pi / 4.0)
    w = math.cos(math.pi / 4.0)
    yaw = quaternion_to_yaw(0.0, 0.0, z, w)
    assert abs(yaw - (math.pi / 2.0)) < 1e-6


def test_yaw_negative_ninety():
    z = -math.sin(math.pi / 4.0)
    w = math.cos(math.pi / 4.0)
    yaw = quaternion_to_yaw(0.0, 0.0, z, w)
    assert abs(yaw + (math.pi / 2.0)) < 1e-6
