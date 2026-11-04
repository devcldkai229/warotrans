from pathlib import Path

for name in (
    "run_dual_semantic_scenarios.py",
    "run_sim01_motion_tour.py",
    "run_speed_zone_velocity_probe.py",
):
    p = Path(__file__).with_name(name)
    if not p.is_file():
        continue
    raw = p.read_bytes()
    raw = raw.replace(b"import Paramete\n", b"import Parameter\n")
    raw = raw.replace(b"TransformListene\n", b"TransformListener\n")
    raw = raw.replace(b"import Parameterer\n", b"import Parameter\n")
    raw = raw.replace(b"TransformListenerer\n", b"TransformListener\n")
    p.write_bytes(raw)
    print(name, "ok")
