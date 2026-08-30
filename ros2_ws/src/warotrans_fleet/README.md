# warotrans_fleet

Owns the high-level boundary between the robot and the fleet/backend system.

Desired direction:

```text
Fleet Manager
   ↓ task/goal
warotrans_fleet
   ↓ navigation goal
Nav2
```

Robot reports:

```text
robot identity
online/offline
pose
battery
task state
navigation state
errors
completion/failure
```

Do not hard-lock the core package to VDA5050 until that protocol is explicitly chosen as an architecture decision.
