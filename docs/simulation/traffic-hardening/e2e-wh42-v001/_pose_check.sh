#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
source /mnt/e/programming/my_project/warotrans/ros2_ws/install/setup.bash
export PYTHONPATH="/home/ser/pydeps${PYTHONPATH:+:$PYTHONPATH}"
python3 - <<'PY'
import json, time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

rclpy.init()
node = Node('pose_check_wh42')
buf = Buffer()
TransformListener(buf, node)
amcl = {'got': False}
odom = {'got': False}

def on_amcl(msg):
    amcl['got'] = True
    amcl['x'] = msg.pose.pose.position.x
    amcl['y'] = msg.pose.pose.position.y

def on_odom(msg):
    odom['got'] = True
    odom['x'] = msg.pose.pose.position.x
    odom['y'] = msg.pose.pose.position.y

node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', on_amcl, 10)
node.create_subscription(Odometry, '/odom', on_odom, 10)

tf_map = None
deadline = time.time() + 20
while time.time() < deadline and (not amcl['got'] or tf_map is None):
    rclpy.spin_once(node, timeout_sec=0.2)
    try:
        t = buf.lookup_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.1))
        tf_map = {
            'x': t.transform.translation.x,
            'y': t.transform.translation.y,
            'z': t.transform.translation.z,
        }
    except Exception:
        pass

result = {
    'amcl': {k: amcl[k] for k in amcl if k != 'got'} if amcl.get('got') else None,
    'odom': {k: odom[k] for k in odom if k != 'got'} if odom.get('got') else None,
    'tf_map_base_link': tf_map,
    'expected_park': {'x': -1.932, 'y': -0.595, 'yaw': -1.5708},
}
print(json.dumps(result, indent=2))
node.destroy_node(); rclpy.shutdown()
PY
