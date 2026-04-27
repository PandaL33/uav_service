# config.py
# RTSP 循环录制配置文件

# ================= 基础设置 =================
# RTSP 流地址 (请修改为你的摄像头地址)
# 格式通常为: rtsp://用户名:密码@IP地址:端口/路径
RTSP_URL = "rtsp://192.168.144.108:554/stream=0"

# 视频保存目录 (如果不存在会自动创建)
# 可以使用相对路径 (如 "./recordings") 或绝对路径 (如 "D:/UAV_Videos")
SAVE_DIR = "./uav_recordings"

# ================= 录制策略 =================
# 每个视频文件的时长 (单位: 秒)
# 例如: 60 = 1分钟一个文件, 300 = 5分钟一个文件
SEGMENT_DURATION = 60

# 最大保留的文件数量
# 当文件数量超过此值时，会自动删除最旧的文件
# 总录像时长 ≈ SEGMENT_DURATION * MAX_FILES
MAX_FILES = 5

# ================= 高级选项 =================
# 强制指定 FPS (帧率)
# 如果摄像头无法正确上报 FPS，或者你想强制降帧保存，可以修改此处
# 设置为 None 则自动检测摄像头 FPS
FORCE_FPS = None 

# 日志级别 (INFO, DEBUG, WARNING)
LOG_LEVEL = "INFO"