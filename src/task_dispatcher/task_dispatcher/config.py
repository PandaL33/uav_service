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

# 实时视频推流断流后的自动重连策略
PREVIEW_RETRY_MAX = 0        # 连续失败重连次数上限；0=不限次数（外网中断恢复前一直重试）
PREVIEW_RETRY_DELAY = 2      # 每次重连固定等待秒数（飞机常进出信号盲区，不逐级递增延时）
PREVIEW_RETRY_RESET_SEC = 30 # 连续运行超过该时长视为稳定，失败计数清零
PREVIEW_STALL_TIMEOUT = 20   # ffmpeg 进度日志超过该秒数无增长即判定卡死（0=关闭检测）

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
