import cv2
import sys

def capture_rtsp_snapshot(rtsp_url, output_filename, timeout=10):
    """
    从 RTSP 流中截取一张图片并保存。

    参数:
    rtsp_url (str): RTSP 流的地址 (例如: rtsp://admin:password@192.168.1.100:554/stream1)
    output_filename (str): 输出图片的文件名 (例如: snapshot.jpg)
    timeout (int): 连接超时时间（秒）
    """
    
    # 1. 创建 VideoCapture 对象
    # 注意：对于某些网络摄像头，可能需要在 URL 后面添加参数以保持连接稳定
    # 例如: rtsp://...?tcp=1 (强制使用 TCP 传输，避免 UDP 丢包导致的绿屏或花屏)
    cap = cv2.VideoCapture(rtsp_url)

    # 设置缓冲区大小（可选，有时能提高读取速度）
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    # 检查摄像头是否成功打开
    if not cap.isOpened():
        print(f"错误: 无法打开 RTSP 流地址: {rtsp_url}")
        return False

    print(f"正在连接到 RTSP 流: {rtsp_url} ...")

    # 2. 预读几帧
    # 网络流通常需要读取前几帧才能获取到清晰的图像，否则第一帧可能是黑的或模糊的
    frame_count = 0
    while frame_count < 20: 
        ret, frame = cap.read()
        if not ret:
            print("警告: 读取帧失败，正在重试...")
            # 如果读取失败，稍微等待一下继续尝试
            continue
        frame_count += 1

    # 3. 正式读取一帧用于截图
    ret, frame = cap.read()

    if ret:
        # 4. 保存图片
        # cv2.IMWRITE_JPEG_QUALITY 设置 JPEG 质量 (0-100)，默认是 95
        cv2.imwrite(output_filename, frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        print(f"成功! 截图已保存为: {output_filename}")
        
        # 获取图片尺寸
        height, width, _ = frame.shape
        print(f"图片分辨率: {width}x{height}")
        return True
    else:
        print("错误: 无法读取视频帧，请检查网络或摄像头状态。")
        return False

    # 5. 释放资源
    cap.release()

# --- 主程序入口 ---
if __name__ == "__main__":
    # 配置 RTSP 地址
    # 示例格式: rtsp://用户名:密码@IP地址:端口/编码通道主码流
    # 海康威视示例: rtsp://admin:12345@192.168.1.64:554/h264/ch1/main/av_stream
    # 大华示例: rtsp://admin:12345@192.168.1.64:554/cam/realmonitor?channel=1&subtype=0
    
    # 请在此处替换为你真实的 RTSP 地址
    TARGET_RTSP_URL = "rtsp://192.168.144.108:554/stream=0"
    
    # 如果不确定具体的流地址，请查阅摄像头厂商的文档
    
    # 检查用户是否修改了默认占位符（防止直接运行报错）
    if "username:password" in TARGET_RTSP_URL:
        print("提示: 请修改代码中的 TARGET_RTSP_URL 变量为你的真实摄像头地址。")
        # 这里为了演示不报错，你可以填入一个测试用的 MP4 文件路径代替 RTSP
        # TARGET_RTSP_URL = "test_video.mp4" 

    # 执行截图
    capture_rtsp_snapshot(TARGET_RTSP_URL, "camera_snapshot.jpg")