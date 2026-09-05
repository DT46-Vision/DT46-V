#!/usr/bin/env bash
# 离线拟合：启动后从 data/ 选择 CSV（也可用: ./fit.sh -- --csv data/xxx.csv）
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
CONFIG="$ROOT/config/fit_config.yaml"
exec python3 "$ROOT/python/fit.py" --config "$CONFIG" "$@"
