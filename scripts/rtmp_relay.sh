#!/usr/bin/env bash
# Sangluwoo 摄像机 RTSP -> RTMP 转发服务 (ARM Ubuntu / RK3588)
#   摄像机 RTSP 拉流, 转封装(或转码)推 RTMP。不依赖设备端协议逆向。
#   所有参数可用环境变量覆盖, 便于 systemd 配置。
set -uo pipefail

# ---------------- 配置 (可被环境变量覆盖) ----------------
RTSP_URL="${RTSP_URL:-}"                         # 显式给出则直接使用, 跳过 IP 探测
RTMP_URL="${RTMP_URL:-rtmp://wrjxk.app.xmschain.com:1935/UAV/UAV00002/19yqM1eMpAVLSpD2S6Ej}"  # 推流地址
MODE="${MODE:-auto}"                             # auto | gst | h264 | copy
SRC_CODEC="${SRC_CODEC:-hevc}"                   # 摄像机源码流编码: hevc | h264, 用于选硬解器/解包器
VIDEO_KBPS="${VIDEO_KBPS:-4000}"                 # 转码码率 (kbps), 太低画面会糊/出块
GOP="${GOP:-50}"                                 # 关键帧间隔
RTP_LATENCY_MS="${RTP_LATENCY_MS:-300}"          # rtspsrc 抖动缓冲(ms); 太小会丢迟到的包导致马赛克
RESTART_EVERY_SEC="${RESTART_EVERY_SEC:-600}"    # 定时重开(秒), 0=关闭; 长时间运行掉码率/花屏时靠它恢复
LOW_KBPS_PCT="${LOW_KBPS_PCT:-40}"               # 实测码率低于目标值的该百分比(且无发送积压)持续 LOW_KBPS_SEC 即重启, 0=关闭
LOW_KBPS_SEC="${LOW_KBPS_SEC:-60}"               # 低码率持续多少秒判定编码异常
RTSP_TRANSPORT="${RTSP_TRANSPORT:-tcp}"          # tcp 更稳, 交换机/NAT 下建议 tcp
RTSP_TIMEOUT_US="${RTSP_TIMEOUT_US:-15000000}"   # 15s 无数据则退出, 交给 systemd 重启
RTSP_TIMEOUT_OPT="${RTSP_TIMEOUT_OPT:-auto}"     # auto 按 ffmpeg 版本选 rw_timeout/stimeout/timeout
FFMPEG_LOGLEVEL="${FFMPEG_LOGLEVEL:-warning}"    # 排查问题可设 info/verbose
WATCHDOG_SEC="${WATCHDOG_SEC:-5}"                # 检查间隔(秒), 0=关闭守护
STALL_SEC="${STALL_SEC:-10}"                     # 连续无数据发出多少秒判定卡死
HEARTBEAT_SEC="${HEARTBEAT_SEC:-30}"             # 正常时的心跳日志间隔(秒)
STARTUP_GRACE_SEC="${STARTUP_GRACE_SEC:-15}"     # 启动宽限期, 期内不判定卡死(等 RTSP 连接/编码器初始化)
LOG_TAG="${LOG_TAG:-rtmp-relay}"
FFMPEG="${FFMPEG:-ffmpeg}"

# ---------------- 摄像机 IP 自动探测 (现场 IP 常变, 按 MAC 定位) ----------------
CAM_MAC="${CAM_MAC:-00:9A:7A:45:00:D7}"          # 目标摄像机 MAC
CAM_IP="${CAM_IP:-}"                             # 手动指定则跳过探测
CAM_IFACE="${CAM_IFACE:-}"                       # 留空自动取默认路由网卡
CAM_SCAN_TIMEOUT="${CAM_SCAN_TIMEOUT:-30}"       # arp-scan/nmap 扫描超时(秒)
CAM_SCAN_WORKERS="${CAM_SCAN_WORKERS:-128}"      # 内置 ping 扫描并发数
CAM_RETRY_SEC="${CAM_RETRY_SEC:-15}"             # 探测不到摄像机时的重试间隔(秒)

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

# 用 arp-scan/nmap 解析 "ip mac" 对应关系(输出到 stdout)。
# 注意: 这两个工具用原始套接字自己发 ARP, 不会写入内核 ARP 表, 因此不能只依赖 ip neigh;
# 它们的结果必须像这里一样从输出里解析。
_tool_hits() {
  local iface="$1" cidr="$2" out n
  if command -v arp-scan >/dev/null 2>&1; then
    out="$(timeout "$CAM_SCAN_TIMEOUT" arp-scan -q --interface="$iface" "$cidr" 2>/dev/null)"
    printf '%s\n' "$out" | awk -F'\t' '$1 ~ /^[0-9]+\./ && $2 ~ /^([0-9a-fA-F]{2}:){5}/ { print $1, tolower($2) }'
    n="$(printf '%s\n' "$out" | grep -cE '^[0-9]+\.')"
    echo "[$LOG_TAG] arp-scan $cidr: 发现 $n 台设备" >&2
  elif command -v nmap >/dev/null 2>&1; then
    out="$(timeout "$CAM_SCAN_TIMEOUT" nmap -sn -n "$cidr" 2>/dev/null)"
    printf '%s\n' "$out" | awk '
      /^Nmap scan report for / { ip = $NF; gsub(/[()]/, "", ip) }
      /^MAC Address: /        { print ip, tolower($3) }'
    n="$(printf '%s\n' "$out" | grep -c 'Nmap scan report')"
    echo "[$LOG_TAG] nmap $cidr: 发现 $n 台在线主机" >&2
  fi
  return 0
}

# 从 "ip mac" 列表里挑出目标 MAC 对应的 IP
_mac_of() {
  printf '%s\n' "$1" | awk -v m="$(printf '%s' "$CAM_MAC" | tr 'A-Z' 'a-z')" '$2 == m { print $1; exit }'
}

# 输出 "网卡 网段" 列表, 默认路由网卡在前; 可用 CAM_IFACE 限定
_iface_cidrs() {
  local def
  def="$(ip -4 route show default 2>/dev/null | awk '{print $5; exit}')"
  if [ -n "$CAM_IFACE" ]; then
    ip -4 -o addr show dev "$CAM_IFACE" up 2>/dev/null | awk '$3 == "inet" { print $2, $4 }'
    return 0
  fi
  [ -n "$def" ] && ip -4 -o addr show dev "$def" up 2>/dev/null | awk -v d="$def" '$3 == "inet" { print d, $4 }'
  # 跳过 docker/veth 等虚拟网卡, 摄像机不会挂在上面
  ip -4 -o addr show up 2>/dev/null |
    awk -v d="$def" '$3 == "inet" && $2 != d && $4 !~ /^127\./ &&
      $2 !~ /^(docker|veth|br-|virbr|tun|tap|vnet|cni|flannel|lo)/ { print $2, $4 }'
}

# 成功时向 stdout 输出 IP, 失败返回非 0
discover_cam_ip() {
  local ip iface cidr myip prefix net round pairs hits

  # 1) 先查 ARP 缓存, 命中且能连通就直接用
  ip="$(_alive_cam_ip)" && { printf '%s\n' "$ip"; return 0; }

  command -v ip >/dev/null 2>&1 || { echo "[$LOG_TAG] 错误: 缺少 ip 命令 (iproute2)" >&2; return 1; }
  pairs="$(_iface_cidrs)"
  [ -n "$pairs" ] || {
    echo "[$LOG_TAG] 错误: 没有可用的 IPv4 网卡或网段 (可用 CAM_IFACE 指定)" >&2
    return 1
  }

  # 2) 对所有本机网段各扫两轮, 覆盖启动慢/丢包的设备, 每轮后按 MAC 反查
  for round in 1 2; do
    while read -r iface cidr; do
      [ -n "$iface" ] || continue
      myip="${cidr%%/*}"; prefix="${cidr##*/}"
      if [ "$prefix" -lt 22 ]; then
        echo "[$LOG_TAG] 网段 /$prefix 过大, 仅在 $myip 所在的 /22 内扫描" >&2
        prefix=22
        net="$myip"
      else
        net="${cidr%%/*}"
      fi
      echo "[$LOG_TAG] 第 $round 轮: 扫描 $iface ($net/$prefix) 查找 $CAM_MAC ..." >&2

      # 2a) ping 扫描: 走内核 ARP, 会真正刷新 ip neigh
      _ping_sweep "$net/$prefix"
      echo "[$LOG_TAG] ping 扫描 $net/$prefix: ARP 表 $(ip -4 neigh show 2>/dev/null | wc -l) 条" >&2

      ip="$(_alive_cam_ip)" && { printf '%s\n' "$ip"; return 0; }

      # 3) 部分摄像机禁 ICMP, ARP 表里有就先用, 由推流结果兜底
      if ip="$(_any_cam_ip)"; then
        echo "[$LOG_TAG] 警告: $ip 无 ICMP 响应, 仍按 ARP 表使用" >&2
        printf '%s\n' "$ip"
        return 0
      fi

      # 2b) arp-scan/nmap 结果不写内核 ARP 表, 需从它们的输出里解析 IP/MAC
      hits="$(_tool_hits "$iface" "$net/$prefix" </dev/null)"
      ip="$(_mac_of "$hits")"
      if [ -n "$ip" ]; then
        echo "[$LOG_TAG] 摄像机 $CAM_MAC -> $ip (来自扫描工具)" >&2
        printf '%s\n' "$ip"
        return 0
      fi
    done <<< "$pairs"
  done

  echo "[$LOG_TAG] 未找到 MAC $CAM_MAC; 已扫描:$(printf '%s\n' "$pairs" | awk '{ printf " %s(%s)", $1, $2 }')" >&2
  echo "[$LOG_TAG] 提示: 摄像机可能与主机不在同一网段(ARP 扫描只能发现同网段设备), 或未上电/未联网; 可给它配置同网段地址, 或用 CAM_IP 手动指定" >&2
  return 1
}

[ -n "$RTMP_URL" ] || { echo "[$LOG_TAG] 错误: 未设置 RTMP_URL" >&2; exit 2; }
command -v "$FFMPEG" >/dev/null 2>&1 || { echo "[$LOG_TAG] 错误: 找不到 ffmpeg" >&2; exit 3; }

# ---------------- 解析摄像机地址 ----------------
if [ -z "$RTSP_URL" ]; then
  if [ -z "$CAM_IP" ]; then
    attempt=0
    # 探测失败不退出, 就地重试; 避免 systemd 反复重启刷计数
    while :; do
      attempt=$((attempt + 1))
      CAM_IP="$(discover_cam_ip)" || CAM_IP=""
      [ -n "$CAM_IP" ] && break
      echo "[$LOG_TAG] 第 ${attempt} 次探测失败, ${CAM_RETRY_SEC}s 后重试" >&2
      sleep "$CAM_RETRY_SEC"
    done
  fi
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

# 编码器是否支持某属性(不同 JetPack 版本有差异, 拼了不存在的属性整条管线起不来)
_gst_enc_prop() {
  gst-inspect-1.0 nvv4l2h264enc 2>/dev/null | grep -qE "^[[:space:]]+$1[[:space:]]+:"
}

# ---------------- 进程守护与状态日志 ----------------
# 推流停止时往往没有任何报错(对端断开/RTSP 静默中断), 这里靠 RTMP 连接的发送字节判断卡死
_child_pid=""
_ts() { date '+%F %T'; }

# 从 URL 取端口(取不到用默认值)
_url_port() {
  local hp="${1#*://}"
  hp="${hp%%/*}"
  case "$hp" in *:*) printf '%s' "${hp##*:}" ;; *) printf '%s' "$2" ;; esac
}
_RTMP_PORT="$(_url_port "$RTMP_URL" 1935)"
_RTSP_PORT="$(_url_port "$RTSP_URL" 554)"

# 统计进程到指定端口那条 TCP 连接: 输出 "peer|sendq|sent|acked|recv", 取不到返回 1。
# 注意: /proc/<pid>/io 的 wchar 只统计文件写, 不含 socket send, 不能用它判断推流是否还在动。
_sock_stats() {
  local port="$1"
  command -v ss >/dev/null 2>&1 || return 1
  # 逐行解析: 状态行取 本端/对端/待发, 紧随其后的 info 行取字节数。
  # 只认"对端端口 == port"的连接, 避免把无关 socket 的字节混进来
  # (曾经把整份 ss 输出当成一个块, 结果取到 127.0.0.1:9090 的 622B 而误杀正常推流)。
  ss -tinp 2>/dev/null | awk -v port="$port" '
    function val(s, pat,   r) {
      if (!match(s, pat)) return 0
      r = substr(s, RSTART, RLENGTH); sub(/^[^:]*:/, "", r); return r + 0
    }
    $1 ~ /^[A-Z][A-Z-]+$/ {
      if ($3 ~ /^[0-9]+$/ && NF >= 5) {
        peer = $5; sendq = $3 + 0; key = $4 " " $5
        hit = (peer ~ (":" port "([^0-9]|$)"))
      } else {
        peer = ""; sendq = 0; hit = 0
      }
    }
    /bytes_(acked|sent|received):/ && hit {
      if (key in seen) next
      seen[key] = 1
      peers = peers (peers == "" ? "" : ",") peer
      total_q += sendq
      sent  += val($0, "bytes_sent:[0-9]+")
      acked += val($0, "bytes_acked:[0-9]+")
      recv  += val($0, "bytes_received:[0-9]+")
      found++
    }
    END { if (found) printf "%s|%d|%d|%d|%d\n", peers, total_q, sent, acked, recv; else exit 1 }'
}

# 排查用: 只打印对端端口匹配的那条连接(状态行 + info 行), 避免整份 ss 刷屏
_sock_debug() {
  local port="$1"
  command -v ss >/dev/null 2>&1 || return 0
  ss -tinp 2>/dev/null | awk -v port="$port" -v tag="$LOG_TAG" '
    $1 ~ /^[A-Z][A-Z-]+$/ {
      hit = ($3 ~ /^[0-9]+$/ && NF >= 5 && $5 ~ (":" port "([^0-9]|$)"))
      if (hit) { gsub(/[ \t]+/, " ", $0); print "[" tag "] 调试: " $0 > "/dev/stderr" }
      next
    }
    /bytes_(acked|sent|received):/ && hit {
      gsub(/[ \t]+/, " ", $0); print "[" tag "] 调试:   " $0 > "/dev/stderr"
    }'
}

trap '_stop_child; echo "[$LOG_TAG] $(_ts) 收到终止信号, 已停止" >&2; exit 0' TERM INT

_stop_child() {
  [ -n "$_child_pid" ] || return 0
  kill -TERM "$_child_pid" 2>/dev/null
  sleep 2
  kill -KILL "$_child_pid" 2>/dev/null
  wait "$_child_pid" 2>/dev/null
  _child_pid=""
}

run_supervised() {
  [ "$WATCHDOG_SEC" -gt 0 ] || { "$@"; return $?; }

  local pid start now last_change rc idle measured=0
  local rt src rp rsq rsent rack rrecv sp ssq ssent sack srecv
  local acked_prev="" src_prev="" out_delta in_delta has_src=0 last_log cur grace_over=0 dumped=0 kbps=0 low_since=""

  echo "[$LOG_TAG] $(_ts) 启动: $*"

  "$@" &
  pid=$!
  _child_pid=$pid
  start=$(date +%s)
  last_change=$start
  last_log=$start

  while kill -0 "$pid" 2>/dev/null; do
    sleep "$WATCHDOG_SEC"
    kill -0 "$pid" 2>/dev/null || break
    now=$(date +%s)
    idle=$((now - last_change))
    # 定时重开: 长时间运行后实际码率会一路下滑且不会自愈, 到点主动重启交给 systemd 拉起
    if [ "$RESTART_EVERY_SEC" -gt 0 ] && [ $((now - start)) -ge "$RESTART_EVERY_SEC" ]; then
      echo "[$LOG_TAG] $(_ts) 已连续运行 $((now - start))s, 按计划每 ${RESTART_EVERY_SEC}s 重启进程" >&2
      _stop_child
      return 7
    fi
    # 启动宽限期内不判定卡死: RTSP 建连 + 硬编初始化可能超过 STALL_SEC
    [ $((now - start)) -ge "$STARTUP_GRACE_SEC" ] && grace_over=1

    rt="$(_sock_stats "$_RTMP_PORT")" || rt=""
    src="$(_sock_stats "$_RTSP_PORT")" || src=""

    # 拉流端(摄像机)收到的字节, 用于区分是摄像机没数据还是推流端没发出去。
    # 仅对 RTSP over TCP 有意义; UDP 传输时这条连接只有控制消息
    if [ -n "$src" ]; then
      IFS='|' read -r sp ssq ssent sack srecv <<< "$src"
      in_delta=$((srecv - ${src_prev:-$srecv}))
      src_prev=$srecv
      has_src=1
    else
      sp="-"; in_delta=0; has_src=0
    fi

    if [ -z "$rt" ]; then
      if [ "$measured" -eq 0 ]; then
        # 尚未观测到推流连接(启动初期或缺少 ss), 不做判断
        echo "[$LOG_TAG] $(_ts) 提示: 运行 $((now - start))s, 未找到推流 TCP 连接, 暂不判断卡死" >&2
      elif [ "$idle" -ge "$STALL_SEC" ] && [ "$grace_over" -eq 1 ]; then
        echo "[$LOG_TAG] $(_ts) 错误: 推流连接已断开 ${idle}s (拉流 $sp 本轮收到 ${in_delta}B), 重启进程" >&2
        _stop_child
        return 7
      else
        echo "[$LOG_TAG] $(_ts) 警告: 推流连接不存在 (${idle}s 无数据, 运行 $((now - start))s, 拉流 $sp 本轮收到 ${in_delta}B)" >&2
      fi
      continue
    fi

    IFS='|' read -r rp rsq rsent rack rrecv <<< "$rt"
    measured=1
    # 以"应用交给内核的字节"(bytes_sent)为主指标, bytes_acked 仅作参考:
    # 部分内核/网卡上 ACK 计数与实际发送不同步, 只看 acked 会误判
    if [ "${rsent:-0}" -gt 0 ]; then cur=$rsent; else cur=${rack:-0}; fi
    out_delta=${acked_prev:+$((cur - acked_prev))}
    # 本轮实测推流码率(按检查间隔折算), 用于心跳展示和码率塌陷判定
    kbps=$(( ${out_delta:-0} * 8 / WATCHDOG_SEC / 1000 ))

    if [ "${out_delta:-0}" -gt 0 ] || { [ -z "$acked_prev" ] && [ "${cur:-0}" -ge 0 ]; }; then
      last_change=$now
      dumped=0
      # 只在首次和每隔 HEARTBEAT_SEC 打一次, 避免检查间隔调小后日志刷屏
      if [ -z "$acked_prev" ] || [ $((now - last_log)) -ge "$HEARTBEAT_SEC" ]; then
        echo "[$LOG_TAG] $(_ts) 心跳: 运行 $((now - start))s, 推流 $rp 已发 ${cur}B(对端确认 ${rack}B)${out_delta:+(本轮 +${out_delta}B, 约 ${kbps}kbps)} 待发 ${rsq}B | 拉流 $sp 本轮收到 ${in_delta}B"
        last_log=$now
      fi
    else
      echo "[$LOG_TAG] $(_ts) 警告: ${idle}s 无数据发出, 推流 $rp 已发 ${cur}B(对端确认 ${rack}B) 待发 ${rsq}B | 拉流 $sp 本轮收到 ${in_delta}B" >&2
      if [ "$dumped" -eq 0 ]; then
        dumped=1
        _sock_debug "$_RTMP_PORT"
        [ "$has_src" -eq 1 ] && _sock_debug "$_RTSP_PORT"
      fi
      if [ "$rsq" -gt 0 ]; then
        echo "[$LOG_TAG] $(_ts) 诊断: 数据积压在内核发送缓冲(待发 ${rsq}B), 对端没有接收" >&2
      elif [ "$has_src" -eq 0 ]; then
        echo "[$LOG_TAG] $(_ts) 诊断: 未找到拉流连接($sp), 无法判断摄像机侧是否有数据" >&2
      elif [ "$in_delta" -eq 0 ]; then
        echo "[$LOG_TAG] $(_ts) 诊断: 摄像机侧也没有新数据到达, 问题在拉流端(RTSP); 若 RTSP 走 UDP, 该统计不适用" >&2
      else
        echo "[$LOG_TAG] $(_ts) 诊断: 拉流正常但编码/推流没有输出, 问题在本地编码或 rtmpsink" >&2
      fi
      if [ "$idle" -ge "$STALL_SEC" ] && [ "$grace_over" -eq 1 ]; then
        echo "[$LOG_TAG] $(_ts) 错误: 连续 ${idle}s 无数据发出, 判定推流已中断, 重启进程" >&2
        _stop_child
        return 7
      fi
    fi

    # 码率塌陷自愈: CBR 下实测码率长期明显低于目标且发送队列没有积压, 说明编码端已经跟不上,
    # 这种情况不会自己恢复; 有积压则属于链路带宽问题, 不在这里重启
    if [ "$LOW_KBPS_PCT" -gt 0 ] && [ "$grace_over" -eq 1 ] && [ "${rsq:-0}" -eq 0 ] && [ "$kbps" -ge 0 ]; then
      if [ "$kbps" -lt $(( VIDEO_KBPS * LOW_KBPS_PCT / 100 )) ]; then
        [ -n "$low_since" ] || low_since=$now
        if [ $((now - low_since)) -ge "$LOW_KBPS_SEC" ]; then
          echo "[$LOG_TAG] $(_ts) 错误: 码率已持续 $((now - low_since))s 低于 $(( VIDEO_KBPS * LOW_KBPS_PCT / 100 ))kbps (当前 ${kbps}kbps), 判定编码异常, 重启进程" >&2
          _stop_child
          return 7
        fi
      else
        low_since=""
      fi
    fi
    acked_prev=$cur
  done

  wait "$pid"
  rc=$?
  _child_pid=""
  echo "[$LOG_TAG] $(_ts) 推流进程退出: 退出码 $rc, 运行 $(( $(date +%s) - start ))s" >&2
  return "$rc"
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
  export GST_DEBUG="${GST_DEBUG:-2}" GST_DEBUG_NO_COLOR=1
  # maxperf-enable: 不让 Jetson 因"负载低"降频, 长时间运行后编码跟不上会导致实际码率一路下滑
  # insert-sps-pps: 定期带上 SPS/PPS, 播放端丢帧后能自己重新同步, 减少持续花屏
  enc=(nvv4l2h264enc bitrate="$((VIDEO_KBPS * 1000))" iframeinterval="$GOP")
  _gst_enc_prop maxperf-enable && enc+=(maxperf-enable=true)
  _gst_enc_prop insert-sps-pps && enc+=(insert-sps-pps=true)
  _gst_enc_prop control-rate && enc+=(control-rate=1)
  # 不同 JetPack 版本支持的属性不同, 打印实际生效的编码参数, 便于远程核对码率为什么上不去
  echo "[$LOG_TAG] 编码参数: ${enc[*]}"
  run_supervised gst-launch-1.0 -e \
    rtspsrc location="$RTSP_URL" protocols="$RTSP_TRANSPORT" latency="$RTP_LATENCY_MS" \
    ! "$depay" ! "$parser" ! nvv4l2decoder ! nvvidconv \
    ! 'video/x-raw(memory:NVMM),format=NV12' \
    ! "${enc[@]}" \
    ! h264parse ! flvmux streamable=true ! rtmpsink location="$RTMP_URL"
  exit $?
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
  -hide_banner -loglevel "$FFMPEG_LOGLEVEL" -nostats
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
run_supervised "$FFMPEG" "${input[@]}" "${enc[@]}" "${output[@]}"
exit $?
