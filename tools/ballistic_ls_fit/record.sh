#!/usr/bin/env bash
# 采数：需已启动自瞄。启动后在终端选择 data/ 下的 CSV（或新建）。
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
WS_SETUP="${WS_SETUP:-}"

set +u
if [[ -n "$WS_SETUP" && -f "$WS_SETUP" ]]; then
  # shellcheck source=/dev/null
  source "$WS_SETUP"
elif [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
  if [[ -f "$HOME/DT46_V/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$HOME/DT46_V/install/setup.bash"
  elif [[ -f "$(cd "$ROOT/../.." && pwd)/install/setup.bash" ]]; then
    # shellcheck source=/dev/null
    source "$(cd "$ROOT/../.." && pwd)/install/setup.bash"
  fi
fi
set -u

GUI="${ENABLE_GUI:-true}"
# 若设置了 CSV_PATH 则跳过交互直接用该文件
export CSV_PATH="${CSV_PATH:-}"

exec python3 "$ROOT/python/record_csv_node.py" --ros-args \
  -p "enable_gui:=${GUI}"
