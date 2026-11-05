import os

#ROBOT_TYPE = "default"
#ROBOT_TYPE = "ffr"
ROBOT_TYPE = "uav"
# 消防机器人接口地址
FFR_SERVER_URL = "http://192.168.200.135:10030"
# 视频流编码格式
CODEC = "h265"
# 视频总开关：true=启用视频预览/录制；false=禁用（仿真环境无 RTSP 相机时置 false，
# 避免对空 URL 探测 RTSP / 起 ffmpeg 刷屏）
ENABLE_VIDEO = True

# 视频RTSP地址
CAMERAID = "camera01"
#CAMERA01_URL = "rtsp://192.168.144.108:554/stream=0"
CAMERA01_URL = "rtsp://192.168.144.25:8554/main.264"
CAMERA02_URL = "" #"rtsp://admin:Lyq%402025@192.168.168.153:554/Streaming/Channels/101"

# 云台控制接口地址
PTZ_URL = "http://192.168.168.153"
PTZ_USERNAME = "admin"
PTZ_PASSWORD = "Lyq@2025"

# 点云数据原始路径
PCD_FILE_PATH = f"{os.path.expanduser('~')}/slam_data/pcd/test.pcd"
PCD_FILE_COMPRESS_PATH = f"{os.path.expanduser('~')}/slam_data/pcd/test_compress.pcd"

AUTH_CONFIG = {
    'USERNAME': 'robot-manage',
    'PASSWORD': 'Ropeok@jf123',
    'TOKEN_ENDPOINT': '/file/provider/v1/file/getUploadTaskToken' # 避免在函数内拼接查询参数
}

# ================= 录制策略 =================
RECORD_FILE_PATH = f"{os.path.expanduser('~')}/slam_data/rtsp"
# 每个视频文件的时长 (单位: 秒)
# 例如: 60 = 1分钟一个文件, 300 = 5分钟一个文件
SEGMENT_DURATION = 300

# 强制指定 FPS (帧率)
# 如果摄像头无法正确上报 FPS，或者你想强制降帧保存，可以修改此处
# 设置为 None 则自动检测摄像头 FPS
FORCE_FPS = None 

# 实时视频推流帧率上限（预览推流按此抽帧，降低机载编码与上行带宽占用）
PREVIEW_FPS = 5

# 预览输出分辨率高度（像素）：宽度按输入比例自动计算并取偶数；0=不缩放，保持相机原始分辨率
# 仅重编码档位（CODEC="h265"）生效；copy 档位无法加滤镜，此处配置不会生效
PREVIEW_HEIGHT = 480

# 实时视频推流断流后的自动重连策略
PREVIEW_RETRY_MAX = 0        # 连续失败重连次数上限；0=不限次数（外网中断恢复前一直重试）
PREVIEW_RETRY_DELAY = 2      # 每次重连固定等待秒数（飞机常进出信号盲区，不逐级递增延时）
PREVIEW_RETRY_RESET_SEC = 30 # 连续运行超过该时长视为稳定，失败计数清零
PREVIEW_START_TIMEOUT = 12   # 播放命令的启动确认窗口秒数：期间进程未退出视为推流链路可用
PREVIEW_IO_TIMEOUT = 10      # ffmpeg 输出侧读写超时秒数（0=不加），断链时让它自行退出触发重连
PREVIEW_ACK_CONTINUE = 100     # 播放命令先回该状态码表示“受理中/继续”，首帧后再补终态 ACK

# 预览推流是否尝试 Rockchip MPP 硬解（输入侧指定 hevc_rkmpp/h264_rkmpp）
# 关闭或平台不支持时自动回退 CPU 软解；编码器同理（h264_rkmpp → libx264）
PREVIEW_HW_DECODE = True

# 高频飞控话题在本节点侧的采样周期（秒）：到点才订阅、收到一条就退订，
# 用少量订阅创建开销换掉 executor 每消息一次的 WaitSet 重建（不动 PX4 侧发布频率）
# 0 或负数 = 常驻订阅（最早的行为）；当前先按最早行为统计，故为 0
ATTITUDE_SAMPLE_PERIOD = 0.0   # /fmu/out/vehicle_attitude：仅供 1Hz 状态上报使用
BATTERY_SAMPLE_PERIOD = 0.0    # /fmu/out/battery_status：供上报与起飞自检取电量/电压

# 是否禁用 rclpy 默认的 QoS 事件处理器（可把重建集从 58 降到 27）
# False = 保留 rclpy 默认行为（最早的行为），当前用于对照统计
DISABLE_QOS_EVENT_CALLBACKS = False

# 性能统计日志：默认关闭。需要现场排查 CPU 时改 True 并重启 task_dispatcher
# 开启后会给 create_subscription 套一层计数包装，并周期性打印一行摘要
ENABLE_PERF_LOG = False
PERF_LOG_INTERVAL = 60     # 摘要打印周期（秒）

# 是否进行起飞前设备状态自检
ENABLE_PREFLIGHT_CHECK = True

# 是否进行起飞前机舱的控制自检
ENABLE_DOCK_CONTROL = False

# 是否进行手动控制巡检任务
ENABLE_MANUAL_CONTROL = False

# 风速阈值
WIND_SPEED_THRESHOLD = 3.0

# 雨量阈值
RAINFALL_THRESHOLD = 0

# ================= 航点失效降级策略 =================
# 连续多少个（不同）航点不可达后触发返航；单个航点会先重试 3 次再跳过
WAYPOINT_CONSEC_LIMIT = 3
