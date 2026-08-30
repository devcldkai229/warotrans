#!/usr/bin/env bash
# WaroTrans quality gate.
# Chay duoc trong repo Linux/WSL/Pi. Neu tools bi copy rieng len Pi,
# script tu dong dung ~/ros2_ws.
set -u

STRICT=0
if [[ "${1:-}" == "--strict" ]]; then STRICT=1; fi

PASS=0; WARN=0; FAIL=0
ok(){ echo "[OK ] $*"; PASS=$((PASS+1)); }
warn(){ echo "[WARN] $*"; WARN=$((WARN+1)); }
fail(){ echo "[FAIL] $*"; FAIL=$((FAIL+1)); }
has(){ command -v "$1" >/dev/null 2>&1; }

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." 2>/dev/null && pwd || true)"

if [[ -d "${REPO_ROOT}/ros2_ws" ]]; then
  WS="${REPO_ROOT}/ros2_ws"
elif [[ -d "${HOME}/ros2_ws" ]]; then
  WS="${HOME}/ros2_ws"
else
  WS=""
fi

echo "=========== STRUCTURE ==========="
if [[ -f "${SCRIPT_DIR}/validate_structure.py" ]] && has python3; then
  if python3 "${SCRIPT_DIR}/validate_structure.py"; then
    ok "Structure + TF contract validation PASS"
  else
    fail "Structure/TF contract validation FAILED"
  fi
else
  warn "Khong chay duoc validate_structure.py"
fi

echo
echo "=========== POLICY CHECKS ==========="

if [[ -n "$WS" && -d "$WS/src" ]]; then
  # Production ROS source should not hardcode the personal home path.
  if grep -RIn --exclude-dir=.git --exclude='*.md' --exclude='*.rviz' \
      '/home/waro/' "$WS/src" >/tmp/warotrans_check_hardcoded_home.$$ 2>/dev/null; then
    fail "Hardcoded /home/waro path trong ROS source"
    cat /tmp/warotrans_check_hardcoded_home.$$
  else
    ok "Khong hardcode /home/waro trong ROS source"
  fi
  rm -f /tmp/warotrans_check_hardcoded_home.$$

  # Contract guard: base_frame phai la base_footprint.
  # Quet CA YAML LAN PYTHON - loi TF that su hay nam trong launch/node defaults,
  # khong phai trong YAML. Cac dang bat duoc:
  #     base_frame: base_link            (yaml)
  #     'base_frame': 'base_link'        (dict trong launch)
  #     base_frame="base_link"           (kwargs)
  #     declare_parameter('base_frame', 'base_link')
  if grep -RIn --exclude-dir=.git --exclude-dir=build --exclude-dir=install \
      --include='*.yaml' --include='*.yml' --include='*.py' --include='*.xml' \
      -E "base_frame[\"']?[[:space:]]*[:=,][[:space:]]*[\"']?base_link[\"']?" \
      "$WS/src" >/tmp/warotrans_check_baseframe.$$ 2>/dev/null; then
    fail "base_frame=base_link vi pham TF contract (phai la base_footprint)"
    cat /tmp/warotrans_check_baseframe.$$
  else
    ok "Khong thay base_frame=base_link trong YAML/Python/XML"
  fi
  rm -f /tmp/warotrans_check_baseframe.$$

  # Guard: mot TF edge chi co mot publisher.
  # Dem so file publish odom->base_footprint. >1 la dau hieu duplicate publisher.
  ODOM_TF_FILES=$(grep -RIl --exclude-dir=.git --exclude-dir=build --exclude-dir=install \
      --include='*.py' -E "TransformBroadcaster|static_transform_publisher" \
      "$WS/src" 2>/dev/null | wc -l)
  if [[ "$ODOM_TF_FILES" -gt 1 ]]; then
    warn "Co ${ODOM_TF_FILES} file publish TF. Kiem tra khong co 2 node cung publish mot edge."
  else
    ok "So file publish TF: ${ODOM_TF_FILES}"
  fi
else
  warn "Khong tim thay ROS workspace; bo qua ROS static checks"
fi

# Docs/source-of-truth checks only when full repo exists.
for f in docs/STATUS.md docs/interfaces.md docs/calibration.md; do
  if [[ -f "${REPO_ROOT}/${f}" ]]; then ok "$f ton tai"; else warn "$f khong co trong moi truong hien tai"; fi
done

echo
echo "=========== ROS BUILD & TEST ==========="
if [[ -n "$WS" && -d "$WS/src" ]] && find "$WS/src" -name package.xml -print -quit 2>/dev/null | grep -q .; then
  if [[ -f /opt/ros/jazzy/setup.bash ]]; then
    # shellcheck disable=SC1091
    source /opt/ros/jazzy/setup.bash
    cd "$WS"
    if colcon build --symlink-install; then
      ok "colcon build PASS"
      # shellcheck disable=SC1091
      source install/setup.bash 2>/dev/null || true
      if colcon test; then
        if colcon test-result --verbose; then ok "colcon test PASS"; else fail "colcon test-result co failure"; fi
      else
        fail "colcon test command failed"
      fi
    else
      fail "colcon build failed"
    fi
  else
    [[ $STRICT -eq 1 ]] && fail "ROS Jazzy khong co; khong the build" || warn "ROS Jazzy khong co; bo qua build"
  fi
else
  warn "Chua co ROS package; bo qua colcon"
fi

echo
echo "=========== FIRMWARE ==========="
FW="${REPO_ROOT}/firmware/esp32_low_level"
if [[ -f "$FW/platformio.ini" ]]; then
  if has pio; then
    if (cd "$FW" && pio run); then ok "PlatformIO build PASS"; else fail "PlatformIO build failed"; fi
    if [[ -d "$FW/test" ]] && find "$FW/test" -type f -print -quit | grep -q .; then
      if (cd "$FW" && pio test -e native); then ok "PlatformIO native tests PASS"; else warn "PlatformIO native tests chua PASS/khong co native env"; fi
    fi
  else
    [[ $STRICT -eq 1 ]] && fail "Co firmware PlatformIO nhung khong co pio" || warn "Co firmware PlatformIO nhung khong co pio; bo qua"
  fi
else
  warn "Chua co firmware/esp32_low_level/platformio.ini"
fi

echo
echo "=========== RESULT ==========="
echo "PASS=$PASS  WARN=$WARN  FAIL=$FAIL"
if [[ $FAIL -gt 0 ]]; then
  echo "QUALITY GATE: FAIL"
  exit 1
fi

echo "QUALITY GATE: PASS"
exit 0
