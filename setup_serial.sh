#!/usr/bin/env bash
#
# setup_serial.sh —— 一键配置 rm_serial / dm_imu 需要的串口 udev 软链接
#
#   生成规则文件: /etc/udev/rules.d/99-fixed-serial.rules
#   目标软链接:   /dev/ttyPortMCU  (rm_serial, 115200)
#                 /dev/ttyPortIMU  (dm_imu,    921600)
#
# 用法:
#   ./setup_serial.sh                              扫描并交互式选择，然后安装
#   ./setup_serial.sh --discover                   只列出当前串口设备及属性
#   ./setup_serial.sh --imu /dev/ttyACM0 --mcu /dev/ttyACM1
#   ./setup_serial.sh --imu ... --mcu ... --print  只打印规则，不改系统
#   ./setup_serial.sh --check                      检查软链接是否已生效
#   ./setup_serial.sh --uninstall                  删除规则文件
#
# 写入 /etc/udev/rules.d/ 需要 root，脚本会自动 sudo 提权。
# 手工步骤说明见 README.md 第 3 节与 src/rm_serial/README.md。

set -euo pipefail

RULES_FILE="${RULES_FILE:-/etc/udev/rules.d/99-fixed-serial.rules}"
SELF="$(readlink -f "${BASH_SOURCE[0]:-$0}")"
ORIG_ARGS=("$@")

IMU_DEV=""
MCU_DEV=""
DRY_RUN=0
DO_DISCOVER=0
DO_CHECK=0
DO_UNINSTALL=0

c_info() { printf '\033[32m[信息]\033[0m %s\n' "$*"; }
c_warn() { printf '\033[33m[警告]\033[0m %s\n' "$*" >&2; }
c_err()  { printf '\033[31m[错误]\033[0m %s\n' "$*" >&2; }

usage() {
  sed -n '3,18p' "$SELF" | sed 's/^# \{0,1\}//'
}

# ---------- 读取设备属性 ----------
prop() {  # $1=设备节点  $2=udev 属性名
  # 末尾 || true: 设备不存在时 udevadm 返回非零，配合 set -o pipefail 会误杀整个脚本
  udevadm info -q property -n "$1" 2>/dev/null | sed -n "s/^$2=//p" | head -n1 || true
}

dev_field() {  # $1=设备节点  $2=字段名
  case "$2" in
    vid)    prop "$1" ID_VENDOR_ID ;;
    pid)    prop "$1" ID_MODEL_ID ;;
    serial) prop "$1" ID_SERIAL_SHORT ;;
    name)   prop "$1" ID_MODEL ;;
    kernel) basename "$1" | sed 's/[0-9]*$//' ;;
    *)      return 1 ;;
  esac
}

list_devices() {
  local d
  for d in /dev/ttyACM* /dev/ttyUSB*; do
    if [ -e "$d" ]; then
      printf '%s\n' "$d"
    fi
  done
}

discover() {
  local dev found=0
  printf '%-16s %-10s %-11s %-22s %s\n' "设备节点" "idVendor" "idProduct" "serial" "型号"
  while IFS= read -r dev; do
    if [ -n "$dev" ]; then
      found=1
      printf '%-16s %-10s %-11s %-22s %s\n' \
        "$dev" \
        "$(dev_field "$dev" vid)" \
        "$(dev_field "$dev" pid)" \
        "$(dev_field "$dev" serial)" \
        "$(dev_field "$dev" name)"
    fi
  done < <(list_devices)
  if [ "$found" -eq 0 ]; then
    c_warn "没有发现 /dev/ttyACM* 或 /dev/ttyUSB* 设备，请插好串口线后重试"
  fi
}

# ---------- 生成单条规则 ----------
build_rule() {  # $1=设备节点  $2=软链接名
  local dev="$1" link="$2"
  local kernel vid pid serial rule
  kernel="$(dev_field "$dev" kernel)"
  vid="$(dev_field "$dev" vid)"
  pid="$(dev_field "$dev" pid)"
  serial="$(dev_field "$dev" serial)"

  if [ -z "$kernel" ]; then
    kernel="ttyACM"
  fi
  if [ -z "$vid" ] || [ -z "$pid" ]; then
    c_warn "$dev 读不到 USB 属性(idVendor/idProduct)，先用占位符 ffff，安装前请务必改成真实值"
    if [ -z "$vid" ]; then vid="ffff"; fi
    if [ -z "$pid" ]; then pid="ffff"; fi
  fi

  rule="ACTION==\"add\", KERNEL==\"${kernel}*\", ATTRS{idVendor}==\"${vid}\", ATTRS{idProduct}==\"${pid}\""
  if [ -n "$serial" ]; then
    rule="$rule, ATTRS{serial}==\"${serial}\""
  else
    c_warn "$dev 没有序列号，只能按 VID/PID 匹配；若同时插两个同型号设备会串号"
  fi
  printf '%s, SYMLINK+="%s", MODE="0666"\n' "$rule" "$link"
}

generate_rules() {
  printf '%s\n' "# 由 setup_serial.sh 生成于 $(date '+%F %T')"
  printf '%s\n' "# 软链接: /dev/ttyPortMCU (rm_serial, 115200) / /dev/ttyPortIMU (dm_imu, 921600)"
  printf '%s\n' "# 生效后需重新插拔 USB 线；验证: ls -l /dev/ttyPortMCU /dev/ttyPortIMU"
  build_rule "$MCU_DEV" ttyPortMCU
  build_rule "$IMU_DEV" ttyPortIMU
}

# ---------- 提权 ----------
need_root() {
  if [ "$(id -u)" -eq 0 ]; then
    return 0
  fi
  if [ "$DRY_RUN" -eq 1 ]; then
    return 0
  fi
  c_info "写入 $RULES_FILE 需要 root 权限，正在提权 (sudo) ..."
  exec sudo -E -- "$SELF" "$@"
}

# ---------- 校验 ----------
verify_links() {
  local link ok=1
  sleep 1
  for link in /dev/ttyPortMCU /dev/ttyPortIMU; do
    if [ -L "$link" ]; then
      c_info "$link -> $(readlink "$link")"
    else
      ok=0
      c_warn "$link 不存在：请把串口线拔下重新插入；仍不行就检查规则里的 VID/PID/serial 是否正确"
    fi
  done
  if [ "$ok" -eq 1 ]; then
    c_info "串口软链接配置完成"
  else
    c_err "串口软链接未全部生效"
  fi
  return 0
}

# ---------- 安装 / 卸载 ----------
install_rules() {  # $1=含规则的临时文件
  local tmp="$1" bak
  if [ -f "$RULES_FILE" ]; then
    bak="${RULES_FILE}.bak.$(date +%Y%m%d%H%M%S)"
    cp -a "$RULES_FILE" "$bak"
    c_info "已备份原规则到 $bak"
  fi
  install -m 0644 -o root -g root "$tmp" "$RULES_FILE"
  c_info "已写入 $RULES_FILE"
  udevadm control --reload-rules
  # 规则写的是 ACTION=="add"，而 udevadm trigger 默认只发 change 事件，这里显式补发 add
  # (有些内核/设备仍需拔插一次 USB 才会稳定生效)
  udevadm trigger --action=add --subsystem-match=tty
  c_info "udev 规则已重新加载"
  verify_links
}

do_uninstall() {
  if [ -f "$RULES_FILE" ]; then
    rm -f "$RULES_FILE"
    c_info "已删除 $RULES_FILE"
    udevadm control --reload-rules
    udevadm trigger
    c_info "请重新插拔 USB 线，软链接随即消失"
  else
    c_info "$RULES_FILE 不存在，无需删除"
  fi
}

select_devices() {
  local -a devs=()
  local d ans default_imu default_mcu
  while IFS= read -r d; do
    if [ -n "$d" ]; then
      devs+=("$d")
    fi
  done < <(list_devices)

  if [ "${#devs[@]}" -eq 0 ]; then
    c_err "没有发现任何串口设备。请插好线后重试，或用 --imu /dev/xxx --mcu /dev/yyy 手动指定"
    exit 1
  fi

  echo "当前发现的串口设备:"
  discover
  echo
  default_imu="${devs[0]}"
  default_mcu="${devs[0]}"
  if [ "${#devs[@]}" -ge 2 ]; then
    default_mcu="${devs[1]}"
  fi

  read -r -p "IMU  (dm_imu, 921600) 用哪个节点? [回车=$default_imu] " ans || true
  IMU_DEV="${ans:-$default_imu}"
  read -r -p "MCU  (rm_serial, 115200) 用哪个节点? [回车=$default_mcu] " ans || true
  MCU_DEV="${ans:-$default_mcu}"
}

# ---------- 主流程 ----------
while [ "$#" -gt 0 ]; do
  case "$1" in
    --imu)       IMU_DEV="${2:-}"; shift 2 ;;
    --mcu)       MCU_DEV="${2:-}"; shift 2 ;;
    --print|-n)  DRY_RUN=1; shift ;;
    --discover)  DO_DISCOVER=1; shift ;;
    --check)     DO_CHECK=1; shift ;;
    --uninstall) DO_UNINSTALL=1; shift ;;
    -h|--help)   usage; exit 0 ;;
    *)           c_err "未知参数: $1"; echo; usage; exit 1 ;;
  esac
done

if [ "$DO_DISCOVER" -eq 1 ]; then
  discover
  exit 0
fi

if [ "$DO_CHECK" -eq 1 ]; then
  verify_links
  exit 0
fi

if [ "$DO_UNINSTALL" -eq 1 ]; then
  need_root "${ORIG_ARGS[@]}"
  do_uninstall
  exit 0
fi

need_root "${ORIG_ARGS[@]}"

if [ -z "$IMU_DEV" ] || [ -z "$MCU_DEV" ]; then
  select_devices
fi

if [ "$IMU_DEV" = "$MCU_DEV" ]; then
  c_err "IMU 与 MCU 不能是同一个设备节点 ($IMU_DEV)，请重新指定"
  exit 1
fi

for d in "$IMU_DEV" "$MCU_DEV"; do
  if [ ! -e "$d" ]; then
    c_warn "$d 当前不存在，读不到属性时会写成占位符 ffff"
  fi
done

TMP_RULES="$(mktemp)"
trap 'rm -f "$TMP_RULES"' EXIT
generate_rules > "$TMP_RULES"

if [ "$DRY_RUN" -eq 1 ]; then
  c_info "以下是将写入 $RULES_FILE 的内容 (--print 模式，未修改系统):"
  echo "------------------------------------------------------------"
  cat "$TMP_RULES"
  echo "------------------------------------------------------------"
  exit 0
fi

install_rules "$TMP_RULES"
