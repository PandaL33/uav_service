#!/usr/bin/env bash
# Sangluwoo 摄像机 RTSP -> RTMP 转发服务 (ARM Ubuntu / RK3588)
#   摄像机 RTSP 拉流, 转封装(或转码)推 RTMP。不依赖设备端协议逆向。
#   所有参数可用环境变量覆盖, 便于 systemd 配置。
set -uo pipefail

# ---------------- 配置 (可被环境变量覆盖) ----------------
RTSP_URL="${RTSP_URL:-}"                         # 显式给出则直接使用, 跳过 IP 探测
RTMP_URL="${RTMP_URL:-rtmp://wrjxk.app.xmschain.com:1935/UAV/UAV00003/19yqM1eMpAVLSpD2S6Ej}"  # 推流地址
MODE="${MODE:-auto}"                             # auto | gst | h264 | copy
SRC_CODEC="${SRC_CODEC:-hevc}"                   # 摄像机源码流编码: hevc | h264, 用于选硬解器/解包器
VIDEO_KBPS="${VIDEO_KBPS:-2000}"                 # 转码码率 (kbps)
GOP="${GOP:-50}"                                 # 关键帧间隔
RTSP_TRANSPORT="${RTSP_TRANSPORT:-tcp}"          # tcp 更稳, 交换机/NAT 下建议 tcp
RTSP_TIMEOUT_US="${RTSP_TIMEOUT_US:-15000000}"   # 15s 无数据则退出, 交给 systemd 重启
RTSP_TIMEOUT_OPT="${RTSP_TIMEOUT_OPT:-auto}"     # auto 按 ffmpeg 版本选 rw_timeout/stimeout/timeout
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

# ---------------- 编解码能力探测 ----------------
encoders="$("$FFMPEG" -hide_banner -encoders 2>/dev/null)"
decoders="$("$FFMPEG" -hide_banner -decoders 2>/dev/null)"
_has() { printf '%s\n' "$1" | grep -qE "[[:space:]]$2[[:space:]]"; }
_is_jetson() { [ -e /etc/nv_tegra_release ] || [ -e /dev/nvhost-msenc ]; }
_gst_nvenc() {
  command -v gst-launch-1.0 >/dev/null 2>&1 &&
    command -v gst-inspect-1.0 >/dev/null 2>&1 &&
    gst-inspect-1.0 nvv4l2h264enc >/dev/null 2>&1
}

# MODE=auto: ffmpeg 自带 NVENC 就用它; 否则 Jetson 上改走 GStreamer(nvv4l2) 硬解硬编
if [ "$MODE" = auto ]; then
  if _has "$encoders" h264_nvv4l2enc; then
    MODE=h264
  elif _is_jetson && _gst_nvenc; then
    MODE=gst
  else
    MODE=h264
  fi
fi

# ---------------- GStreamer 路径 (Jetson: NVDEC 解码 + NVENC 编码) ----------------
if [ "$MODE" = gst ]; then
  _gst_nvenc || { echo "[$LOG_TAG] 错误: 缺少 gst-launch-1.0 或 nvv4l2h264enc 插件" >&2; exit 6; }
  case "$SRC_CODEC" in
    h264) depay=rtph264depay; parser=h264parse ;;
    *)    depay=rtph265depay; parser=h265parse ;;
  esac
  echo "[$LOG_TAG] 开始: $RTSP_URL -> $RTMP_URL  (gst: nvv4l2decoder + nvv4l2h264enc ${VIDEO_KBPS}kbps)"
  exec gst-launch-1.0 -e \
    rtspsrc location="$RTSP_URL" protocols="$RTSP_TRANSPORT" latency=100 \
    ! "$depay" ! "$parser" ! nvv4l2decoder ! nvvidconv \
    ! 'video/x-raw(memory:NVMM),format=NV12' \
    ! nvv4l2h264enc bitrate="$((VIDEO_KBPS * 1000))" iframeinterval="$GOP" \
    ! h264parse ! flvmux streamable=true ! rtmpsink location="$RTMP_URL"
fi

# ---------------- ffmpeg 路径 ----------------
# 不同 ffmpeg 版本对 RTSP 超时选项的命名不同(老版本只有 stimeout), 不支持则不加
_rtsp_timeout_opt() {
  local help opt
  case "$RTSP_TIMEOUT_OPT" in
    auto)
      help="$("$FFMPEG" -hide_banner -h demuxer=rtsp 2>/dev/null)"
      for opt in rw_timeout stimeout timeout; do
        if printf '%s\n' "$help" | grep -qE "^[[:space:]]*-${opt}[[:space:]]"; then
          printf '%s' "$opt"
          return 0
        fi
      done
      return 0
      ;;
    none) return 0 ;;
    *) printf '%s' "$RTSP_TIMEOUT_OPT" ;;
  esac
}

timeout_opt="$(_rtsp_timeout_opt)"
timeout_args=()
[ -n "$timeout_opt" ] && timeout_args=(-"$timeout_opt" "$RTSP_TIMEOUT_US")

dec_args=(); dec_name="auto"
enc=(); enc_name="copy"

case "$MODE" in
  copy)
    # 不转码, CPU 最低。仅当摄像机输出 H.264 时可用; 源为 HEVC/H.265 时 FLV 不支持, 会报
    # "Video codec hevc not compatible with flv", 请用 MODE=h264。
    enc=(-c:v copy -c:a copy -fflags +genpts)
    ;;
  h264)
    # 硬解: 可用则显式指定解码器, 否则交给 ffmpeg 自动选择
    if _has "$decoders" "${SRC_CODEC}_nvv4l2dec"; then
      dec_args=(-c:v "${SRC_CODEC}_nvv4l2dec")
      dec_name="${SRC_CODEC}_nvv4l2dec"
    fi

    # 硬编优先: Jetson NVENC -> RK3588 V4L2 (Jetson 上 V4L2 无可用设备, 跳过) -> libx264
    if _has "$encoders" h264_nvv4l2enc; then
      enc_name=h264_nvv4l2enc
    elif _has "$encoders" h264_v4l2m2m && ! _is_jetson; then
      enc_name=h264_v4l2m2m
    elif _has "$encoders" libx264; then
      enc_name=libx264
    else
      echo "[$LOG_TAG] 错误: 当前 ffmpeg 没有可用的 H.264 编码器 (可用 MODE=gst 走 NVENC)" >&2
      exit 5
    fi

    if [ "$enc_name" = libx264 ]; then
      enc=(-c:v libx264 -preset ultrafast -tune zerolatency -b:v "${VIDEO_KBPS}k" -g "$GOP"
           -c:a aac -ar 44100 -ac 1 -b:a 64k)
    else
      enc=(-c:v "$enc_name" -b:v "${VIDEO_KBPS}k" -g "$GOP" -c:a aac -ar 44100 -ac 1 -b:a 64k)
    fi
    ;;
  *)
    echo "[$LOG_TAG] 未知 MODE=$MODE (应为 auto/gst/h264/copy)" >&2
    exit 2
    ;;
esac

# ---------------- 输入选项 ----------------
input=(
  -hide_banner -loglevel warning -nostats
  -rtsp_transport "$RTSP_TRANSPORT"
  "${timeout_args[@]}"
  "${dec_args[@]}"
  -i "$RTSP_URL"
)

# ---------------- 输出 ----------------
output=(
  -f flv
  -flvflags no_duration_filesize
  "$RTMP_URL"
)

echo "[$LOG_TAG] 开始: $RTSP_URL -> $RTMP_URL  (mode=$MODE, 解码=$dec_name, 编码=$enc_name, transport=$RTSP_TRANSPORT)"
exec "$FFMPEG" "${input[@]}" "${enc[@]}" "${output[@]}"
