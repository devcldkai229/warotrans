#!/usr/bin/env python3
"""Test publisher đóng vai Backend: gửi NAVIGATE_TO_POSE / CANCEL và in ACK, RESULT, telemetry.

Payload đúng contract warotrans-system docs/mqtt-robot-contracts.md. Cần paho-mqtt.

  python3 fleet_command_cli.py --host localhost --robot RBT-001 navigate --x 1.0 --y 0.5 --yaw 0
  python3 fleet_command_cli.py --robot RBT-001 navigate --x 1.0 --y 0.5 --yaw 1.5708 --timeout 120
  python3 fleet_command_cli.py --robot RBT-001 cancel --target <commandId>

Navigate chờ tới khi có RESULT; Ctrl+C trong lúc chờ sẽ gửi CANCEL cho command đó.
"""

from __future__ import annotations

import argparse
import json
import sys
import threading
import time
import uuid
from datetime import datetime, timezone

import paho.mqtt.client as mqtt


def _now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")


def _command(robot: str, command_type: str, payload: dict) -> dict:
    return {
        "schemaVersion": 1,
        "messageId": str(uuid.uuid4()),
        "commandId": str(uuid.uuid4()),
        "robotCode": robot,
        "type": command_type,
        "issuedAt": _now(),
        "jobAssignmentId": None,
        "jobStepId": None,
        "payload": payload,
    }


def _client(client_id: str) -> mqtt.Client:
    try:
        return mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=client_id)
    except AttributeError:  # paho-mqtt < 2.0
        return mqtt.Client(client_id=client_id)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot", default="RBT-001")
    parser.add_argument("--prefix", default="warotrans/v1")
    parser.add_argument("--quiet-telemetry", action="store_true")
    sub = parser.add_subparsers(dest="cmd", required=True)

    nav = sub.add_parser("navigate")
    nav.add_argument("--x", type=float, required=True)
    nav.add_argument("--y", type=float, required=True)
    nav.add_argument("--yaw", type=float, required=True, help="radian")
    nav.add_argument("--frame", default="map")
    nav.add_argument("--timeout", type=float, default=None, help="timeoutSeconds (optional)")
    nav.add_argument("--map", default=None, help="mapVersionCode (optional)")

    can = sub.add_parser("cancel")
    can.add_argument("--target", required=True, help="commandId đang chạy")

    args = parser.parse_args()
    base = f"{args.prefix}/robots/{args.robot}"

    if args.cmd == "navigate":
        payload = {"frameId": args.frame, "x": args.x, "y": args.y, "yaw": args.yaw}
        if args.timeout is not None:
            payload["timeoutSeconds"] = args.timeout
        if args.map:
            payload["mapVersionCode"] = args.map
        command = _command(args.robot, "NAVIGATE_TO_POSE", payload)
    else:
        command = _command(args.robot, "CANCEL", {"targetCommandId": args.target})

    done = threading.Event()
    subscribed = threading.Event()
    last_tel = [0.0]

    def on_connect(client, _u, _f, rc, *_a):
        client.subscribe([(f"{base}/command_ack", 1), (f"{base}/command_result", 1), (f"{base}/telemetry", 0)])
        subscribed.set()

    def on_message(_c, _u, msg):
        body = json.loads(msg.payload.decode("utf-8"))
        leaf = msg.topic.rsplit("/", 1)[-1]
        if leaf == "telemetry":
            if args.quiet_telemetry or time.monotonic() - last_tel[0] < 1.0:
                return
            last_tel[0] = time.monotonic()
            print(
                f"  telemetry nav={body.get('navigationStatus')} loc={body.get('localizationStatus')} "
                f"cmd={body.get('currentCommandId')} pose={body.get('pose')}"
            )
            return
        if body.get("commandId") not in (command["commandId"], args.__dict__.get("target")):
            return
        print(f"<- {leaf}: {json.dumps(body, indent=2)}")
        if leaf == "command_ack" and not body.get("accepted"):
            done.set()
        if leaf == "command_result" and body.get("commandId") == command["commandId"]:
            done.set()

    client = _client(f"warotrans-cli-{uuid.uuid4().hex[:8]}")
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(args.host, args.port, keepalive=30)
    client.loop_start()
    if not subscribed.wait(5.0):
        print("MQTT connect/subscribe timeout", file=sys.stderr)
        return 2

    print(f"-> {base}/command: {json.dumps(command, indent=2)}")
    client.publish(f"{base}/command", json.dumps(command), qos=1).wait_for_publish()

    try:
        done.wait()
    except KeyboardInterrupt:
        if args.cmd == "navigate":
            cancel = _command(args.robot, "CANCEL", {"targetCommandId": command["commandId"]})
            print(f"\n-> CANCEL {command['commandId']}")
            client.publish(f"{base}/command", json.dumps(cancel), qos=1).wait_for_publish()
            try:
                done.wait(30.0)
            except KeyboardInterrupt:
                pass
    client.loop_stop()
    client.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
