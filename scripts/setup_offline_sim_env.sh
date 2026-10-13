#!/usr/bin/env bash
# 仿真离线航线（uav_service/src/task_dispatcher/task_dispatcher/offline_sim）依赖准备脚本
# —— 只负责装本机缺的 Python/系统依赖，不改动任何在线代码。
#
# 背景（本机实测缺口）：
#   paho.mqtt  缺失   -> ros2_topic_subscriber / rtsp_record_manager 模块级导入
#   gpiod      缺失   -> uav_action_manager 模块级导入（仅 import，仿真不使用 GPIO）
#   cv2        不可用 -> rtsp_record_manager 模块级导入（系统 numpy 2.2.6 与 cv2 ABI 冲突）
#
# 说明：robot_interface / multi_algo_interfaces 是本仓 src/ 下的 ROS 接口包，需由 ROS 工作区
#      构建一次后才能被在线/离线代码 import；本脚本不负责构建它们，只装本机缺失的 Python/系统依赖。
#
# 用法：
#   bash uav_service/scripts/setup_offline_sim_env.sh            # 建 venv + 装 python 依赖
#   bash uav_service/scripts/setup_offline_sim_env.sh --skip-apt # 跳过 sudo apt（gpiod 自行处理）
#   bash uav_service/scripts/setup_offline_sim_env.sh --system   # 不建 venv，直接 pip install --user
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"          # <repo> = .../XTDrone2_ego_planner
WS_ROOT="$(cd "${REPO_ROOT}/.." && pwd)"                 # <ws>   = .../xtd2_ws
VENV_DIR="${REPO_ROOT}/uav_service/.venv"
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="${WS_ROOT}/install/setup.bash"
USE_WS_ROOT="${WS_SETUP}"

SKIP_APT=0
USE_SYSTEM_PY=0
for arg in "$@"; do
  case "$arg" in
    --skip-apt) SKIP_APT=1 ;;
    --system)   USE_SYSTEM_PY=1 ;;
    -h|--help)  sed -n '1,20p' "$0"; exit 0 ;;
    *) echo "未知参数: $arg（--skip-apt / --system）" >&2; exit 2 ;;
  esac
done

step() { echo -e "\n\033[1;36m==> $*\033[0m"; }
warn() { echo -e "\033[1;33m[warn]\033[0m $*"; }
die()  { echo -e "\033[1;31m[error]\033[0m $*" >&2; exit 1; }

# ROS/colcon 的 setup.bash 里会读未定义变量（如 AMENT_TRACE_SETUP_FILES），
# 与本脚本的 set -u 冲突，所以 source 前后临时关掉 nounset。
safe_source() {
  local rc=0
  set +u
  # shellcheck disable=SC1090
  source "$1" || rc=$?
  set -u
  [ "${rc}" -eq 0 ] || die "source $1 失败(exit=${rc})"
}

[ -f "${ROS_SETUP}" ] || die "未找到 ${ROS_SETUP}，本脚本仅针对 ROS2 Humble"

# ---------------------------------------------------------------------------
# 1) 系统包：python3-libgpiod（提供 `import gpiod`，uav_action_manager 只在顶层 import）
# ---------------------------------------------------------------------------
step "检查系统依赖 python3-libgpiod"
if python3 -c "import gpiod" >/dev/null 2>&1; then
  echo "gpiod 可用，跳过"
elif [ "${SKIP_APT}" -eq 1 ]; then
  warn "--skip-apt：请自行安装后重试： sudo apt install -y python3-libgpiod"
else
  step "sudo apt install python3-libgpiod（uav_action_manager 的顶层 import 需要）"
  sudo apt-get install -y python3-libgpiod || warn "安装失败，可手动执行： sudo apt install -y python3-libgpiod"
fi

# ---------------------------------------------------------------------------
# 2) Python 依赖：venv(--system-site-packages) 内锁 numpy<2 + paho-mqtt + opencv
# ---------------------------------------------------------------------------
PY_BIN="python3"
if [ "${USE_SYSTEM_PY}" -eq 0 ]; then
  step "创建离线仿真 venv: ${VENV_DIR}（继承 ROS site-packages，不动系统 numpy）"
  if [ ! -x "${VENV_DIR}/bin/python3" ]; then
    python3 -m venv --system-site-packages "${VENV_DIR}"
  else
    echo "venv 已存在，复用"
  fi
  PY_BIN="${VENV_DIR}/bin/python3"
  PIP_ARGS=()
else
  step "--system：直接装到用户 site-packages（注意会把 numpy 降到 <2，可能影响其它节点）"
  PIP_ARGS=(--user)
fi

step "安装/锁定 Python 依赖：numpy<2, paho-mqtt, opencv-python-headless"
"${PY_BIN}" -m pip install -q "${PIP_ARGS[@]+"${PIP_ARGS[@]}"}" --upgrade pip
"${PY_BIN}" -m pip install "${PIP_ARGS[@]+"${PIP_ARGS[@]}"}" \
  "numpy<2" "paho-mqtt>=1.6" "opencv-python-headless"

# ---------------------------------------------------------------------------
# 3) 自检：离线链路所需 Python 模块能否导入（需在 ROS/工作区已 source 的环境里）
#    接口包(robot_interface/multi_algo_interfaces)属 ROS 包，由工作区 colcon 构建，不在本脚本职责内
# ---------------------------------------------------------------------------
step "导入自检"
safe_source "${ROS_SETUP}"
if [ -f "${WS_SETUP}" ]; then
  safe_source "${WS_SETUP}"
else
  warn "未找到 ${WS_SETUP}，工作区接口包可能不在 PYTHONPATH 里"
fi
REPO_ROOT_ENV="${REPO_ROOT}" "${PY_BIN}" - <<'PYEOF' || die "导入自检失败，请按上面提示补齐依赖后重跑本脚本"
import importlib
import os
import sys

# 让 task_dispatcher 包，以及它内部混用的顶层导入（如 `from uav_command_sender import ...`）都能解析
mod_root = os.path.join(os.environ['REPO_ROOT_ENV'], 'uav_service', 'src',
                        'task_dispatcher', 'task_dispatcher')
for p in (mod_root, os.path.dirname(mod_root)):
    if p not in sys.path:
        sys.path.insert(0, p)

need = ['rclpy', 'paho.mqtt.client', 'gpiod', 'tf_transformations', 'cv2',
        'px4_msgs.msg', 'numpy']
bad = []
for m in need:
    try:
        importlib.import_module(m)
        print('OK     ', m)
    except Exception as e:
        print('MISSING', m, type(e).__name__, e)
        bad.append(m)
if bad:
    raise SystemExit('缺少模块: ' + ', '.join(bad))
print('\nPython 依赖就绪。若 import 缺 robot_interface/multi_algo_interfaces，'
      '请先在工作区 colcon build 本仓 src/ 下的接口包。')
PYEOF

step "完成"
cat <<EOF
后续使用（同一终端里先 source 再跑）：
  source ${ROS_SETUP}
  source ${USE_WS_ROOT}
  # 1) 起仿真（另开终端）
  ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
  # 2) 起离线航线（只起调度侧，就绪后立刻执行航线；默认读 launch 目录旁的 waypoints_3_3.json）
  ros2 launch ${REPO_ROOT}/uav_service/src/task_dispatcher/launch/offline_route_sim.launch.py
  # 换航线：route_file:=<路径或裸文件名>；只看报文不真飞：追加 dry_run:=true
  # 上报落盘：/tmp/uav_offline_sim/action_report.jsonl
EOF
