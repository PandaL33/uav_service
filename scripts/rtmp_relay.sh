#!/usr/bin/env bash
# Sangluwoo 摄像机 RTSP -> RTMP 转发服务 (ARM Ubuntu / RK3588)
#   摄像机 RTSP 拉流, 转封装(或转码)推 RTMP。不依赖设备端协议逆向。
#   所有参数可用环境变量覆盖, 便于 systemd 配置。
set -uo pipefail

# ---------------- 配置 (可被环境变量覆盖) ----------------
RTSP_URL="${RTSP_URL:-}"                         # 显式给出则直接使用, 跳过 IP 探测
RTMP_URL="${RTMP_URL:-}"                         # 必填: rtmp://服务器/app/直播码
MODE="${MODE:-copy}"                             # copy=直转低CPU | h264=转码兼容性好
RTSP_TRANSPORT="${RTSP_TRANSPORT:-tcp}"          # tcp 更稳, 交换机/NAT 下建议 tcp
RTSP_TIMEOUT_US="${RTSP_TIMEOUT_US:-15000000}"   # 15s 无数据则退出, 交给 systemd 重启
LOG_TAG="${LOG_TAG:-rtmp-relay}"
FFMPEG="${FFMPEG:-ffmpeg}"

# ---------------- 摄像机 IP 自动探测 (现场 IP 常变, 按 MAC 定位) ----------------
CAM_MAC="${CAM_MAC:-00:9A:7A:45:00:D7}"          # 目标摄像机 MAC
CAM_IP="${CAM_IP:-}"                             # 手动指定则跳过探测
CAM_IFACE="${CAM_IFACE:-}"                       # 留空自动取默认路由网卡
CAM_SCAN_TIMEOUT="${CAM_SCAN_TIMEOUT:-30}"       # arp-scan/nmap 扫描超时(秒)
CAM_SCAN_WORKERS="${CAM_SCAN_WORKERS:-128}"      # 内置 ping 扫描并发数

RTSP_USER="${RTSP_USER:-admin}"
RTSP_PASS="${RTSP_PASS:-123456}"
RTSP_PORT="${RTSP_PORT:-554}"
RTSP_PATH="${RTSP_PATH:-/mpeg4}"

_ip2int() { local IFS=.; read -r a b c d <<<"$1"; echo $(( (a << 24) | (b << 16) | (c << 8) | d )); }

# 从 ARP(neighbour) 表中按 MAC 查 IP, 大小写不敏感
_neigh_ip_by_mac() {
  ip -4 neigh show 2>/dev/null | awk -v m="$CAM_MAC" '
    BEGIN { m = tolower(m) }
    $NF != "FAILED" && $NF != "INCOMPLETE" {
      for (i = 1; i <= NF; i++) if (tolower($i) == m) { print $1 }
    }'
}

# 优先返回 ICMP 可达的地址
_alive_cam_ip() {
  local ip
  for ip in $(_neigh_ip_by_mac); do
    if _ip_alive "$ip"; then printf '%s\n' "$ip"; return 0; fi
  done
  return 1
}

# 退路: 部分设备禁 ICMP, 只要 ARP 表里有就认
_any_cam_ip() {
  local ips
  ips="$(_neigh_ip_by_mac)"
  [ -n "$ips" ] || return 1
  printf '%s\n' "$ips" | head -n 1
}

_ip_alive() { ping -c 1 -W 1 -n -q "$1" >/dev/null 2>&1; }

# 并发 ping 网段内主机, 借此刷新 ARP 表 (无 arp-scan/nmap 时使用)
_ping_sweep() {
  local cidr="$1" net prefix mask start end i
  net="${cidr%%/*}"; prefix="${cidr##*/}"
  mask=$(( (0xFFFFFFFF << (32 - prefix)) & 0xFFFFFFFF ))
  start=$(( ($(_ip2int "$net") & mask) + 1 ))
  end=$(( start + (1 << (32 - prefix)) - 3 ))
  for (( i = start; i <= end; i++ )); do
    printf '%d.%d.%d.%d\n' \
      $(( (i >> 24) & 255 )) $(( (i >> 16) & 255 )) $(( (i >> 8) & 255 )) $(( i & 255 ))
  done | xargs -r -P "$CAM_SCAN_WORKERS" -n 1 ping -c 1 -W 1 -n -q >/dev/null 2>&1
}

_scan_subnet() {
  local iface="$1" cidr="$2"
  if command -v arp-scan >/dev/null 2>&1; then
    timeout "$CAM_SCAN_TIMEOUT" arp-scan -q --interface="$iface" "$cidr" >/dev/null 2>&1
  elif command -v nmap >/dev/null 2>&1; then
    timeout "$CAM_SCAN_TIMEOUT" nmap -sn -n "$cidr" >/dev/null 2>&1
  else
    _ping_sweep "$cidr"
  fi
  return 0
}

# 成功时向 stdout 输出 IP, 失败返回非 0
discover_cam_ip() {
  local ip iface cidr myip prefix net

  # 1) 先查 ARP 缓存, 命中且能连通就直接用
  ip="$(_alive_cam_ip)" && { printf '%s\n' "$ip"; return 0; }

  # 2) 确定扫描网卡与网段
  command -v ip >/dev/null 2>&1 || { echo "[$LOG_TAG] 错误: 缺少 ip 命令 (iproute2)" >&2; return 1; }
  iface="${CAM_IFACE:-$(ip -4 route show default 2>/dev/null | awk '{print $5; exit}')}"
  [ -n "$iface" ] || { echo "[$LOG_TAG] 错误: 找不到默认路由网卡, 可用 CAM_IFACE 指定" >&2; return 1; }
  cidr="$(ip -4 -o addr show dev "$iface" 2>/dev/null | awk '$3 == "inet" { print $4; exit }')"
  [ -n "$cidr" ] || { echo "[$LOG_TAG] 错误: 网卡 $iface 没有 IPv4 地址" >&2; return 1; }

  myip="${cidr%%/*}"; prefix="${cidr##*/}"
  if [ "$prefix" -ge 22 ]; then
    net="${cidr%%/*}"
  else
    echo "[$LOG_TAG] 网段 /$prefix 过大, 仅在 $myip 所在的 /22 内扫描" >&2
    prefix=22
    net="$myip"
  fi

  # 3) 主动扫描刷新 ARP 表, 再按 MAC 反查
  echo "[$LOG_TAG] ARP 表未命中 $CAM_MAC, 开始扫描 $iface ($net/$prefix) ..." >&2
  _scan_subnet "$iface" "$net/$prefix"

  ip="$(_alive_cam_ip)" && { printf '%s\n' "$ip"; return 0; }

  # 4) 部分摄像机禁 ICMP, ARP 表里有就先用, 由 ffmpeg 拉流结果兜底
  if ip="$(_any_cam_ip)"; then
    echo "[$LOG_TAG] 警告: $ip 无 ICMP 响应, 仍按 ARP 表使用" >&2
    printf '%s\n' "$ip"
    return 0
  fi

  echo "[$LOG_TAG] 未找到 MAC $CAM_MAC 对应的主机" >&2
  return 1
}

[ -n "$RTMP_URL" ] || { echo "[$LOG_TAG] 错误: 未设置 RTMP_URL" >&2; exit 2; }
command -v "$FFMPEG" >/dev/null 2>&1 || { echo "[$LOG_TAG] 错误: 找不到 ffmpeg" >&2; exit 3; }

# ---------------- 解析摄像机地址 ----------------
if [ -z "$RTSP_URL" ]; then
  if [ -z "$CAM_IP" ]; then
    CAM_IP="$(discover_cam_ip)" || CAM_IP=""
  fi
  [ -n "$CAM_IP" ] || {
    echo "[$LOG_TAG] 错误: 无法按 MAC $CAM_MAC 定位摄像机 IP (可用 CAM_IP 手动指定)" >&2
    exit 4
  }
  RTSP_URL="rtsp://${RTSP_USER}:${RTSP_PASS}@${CAM_IP}:${RTSP_PORT}${RTSP_PATH}"
  echo "[$LOG_TAG] 摄像机 $CAM_MAC -> $CAM_IP"
fi

# ---------------- 输入选项 ----------------
input=(
  -hide_banner -loglevel warning -nostats
  -rtsp_transport "$RTSP_TRANSPORT"
  -rw_timeout "$RTSP_TIMEOUT_US"
  -i "$RTSP_URL"
)

# ---------------- 编解码选项 ----------------
case "$MODE" in
  copy)
    # 不转码, CPU 最低。要求 RTMP 服务器支持 MPEG-4 Part 2 视频编码。
    # 大多数云直播(阿里云/腾讯云/抖音)只认 H.264, 那种场景请用 MODE=h264。
    enc=(-c:v copy -c:a copy -fflags +genpts)
    ;;
  h264)
    # RK3588 优先用硬件编码器 h264_v4l2m2m, 缺失则回退 libx264 ultrafast。
    if "$FFMPEG" -hide_banner -encoders 2>/dev/null | grep -q 'h264_v4l2m2m'; then
      enc=(-c:v h264_v4l2m2m -b:v 2000k -g 50 -c:a aac -ar 44100 -ac 1 -b:a 64k)
    else
      enc=(-c:v libx264 -preset ultrafast -tune zerolatency -b:v 2000k -g 50
           -c:a aac -ar 44100 -ac 1 -b:a 64k)
    fi
    ;;
  *)
    echo "[$LOG_TAG] 未知 MODE=$MODE (应为 copy 或 h264)" >&2
    exit 2
    ;;
esac

# ---------------- 输出 ----------------
output=(
  -f flv
  -flvflags no_duration_filesize
  "$RTMP_URL"
)

echo "[$LOG_TAG] 开始: $RTSP_URL -> $RTMP_URL  (mode=$MODE, transport=$RTSP_TRANSPORT)"
exec "$FFMPEG" "${input[@]}" "${enc[@]}" "${output[@]}"
