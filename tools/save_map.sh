#!/usr/bin/env bash
# Wrapper mỏng quanh nav2_map_server map_saver_cli.
# Chạy trên Raspberry Pi trong lúc slam_toolbox vẫn đang publish /map:
#   bash tools/save_map.sh                    # -> ~/maps/warotrans
#   bash tools/save_map.sh ~/maps/kho_tang_1  # tên khác
#   bash tools/save_map.sh ~/maps/warotrans --force
#
# Script này KHÔNG tự tune, KHÔNG tự đổi map, và KHÔNG ghi đè im lặng.
set -euo pipefail

MAP_STEM="${1:-${HOME}/maps/warotrans}"
FORCE="${2:-}"

# map_saver_cli tự thêm đuôi .pgm/.yaml vào stem.
PGM="${MAP_STEM}.pgm"
YAML="${MAP_STEM}.yaml"

mkdir -p "$(dirname "${MAP_STEM}")"

if [[ -e "${PGM}" || -e "${YAML}" ]]; then
  if [[ "${FORCE}" != "--force" ]]; then
    echo "REFUSING TO OVERWRITE: đã tồn tại ${PGM} hoặc ${YAML}" >&2
    echo "Dùng tên khác, hoặc thêm --force nếu thực sự muốn ghi đè." >&2
    exit 1
  fi
  echo "WARNING: --force được bật, sẽ ghi đè ${MAP_STEM}.{pgm,yaml}" >&2
fi

echo "Saving map to ${MAP_STEM}.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli \
  -f "${MAP_STEM}" \
  --ros-args -p save_map_timeout:=10000.0

echo
echo "Kết quả:"
ls -l "${PGM}" "${YAML}"
