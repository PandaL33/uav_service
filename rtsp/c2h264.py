import subprocess
import os
import sys

def convert_to_h264(input_file, output_file=None, remove_audio=False):
    """
    将视频转码为 H.264 编码的 MP4 文件
    
    Args:
        input_file (str): 输入视频路径
        output_file (str): 输出视频路径，默认为 input_file + "_h264.mp4"
        remove_audio (bool): 是否移除音频，默认保留
    """
    if not os.path.exists(input_file):
        print(f"错误: 文件 {input_file} 不存在")
        return False

    if output_file is None:
        base_name = os.path.splitext(input_file)[0]
        output_file = f"{base_name}_h264.mp4"

    # 构建 FFmpeg 命令
    # -i: 输入文件
    # -c:v libx264: 视频编码为 H.264
    # -preset veryfast: 编码速度预设 (越快压缩率越低，可选 ultrafast, fast, medium, slow)
    # -crf 23: 恒定速率因子 (0-51, 数值越小质量越高文件越大，23是默认值，18-28通常用于高清)
    # -c:a copy: 音频直接复制 (不重新编码，节省时间)
    # -y: 覆盖输出文件
    
    cmd = [
        'ffmpeg',
        '-i', input_file,
        '-c:v', 'libx264',
        '-preset', 'veryfast',
        '-crf', '23',
        '-pix_fmt', 'yuv420p', # 确保兼容性 (H.264 标准通常要求 4:2:0)
    ]

    if remove_audio:
        cmd.append('-an')
    else:
        cmd.extend(['-c:a', 'copy'])
        
    cmd.extend(['-y', output_file])

    print(f"正在转码: {input_file} -> {output_file}")
    try:
        # 执行命令
        subprocess.run(cmd, check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        print("转码成功！")
        return True
    except subprocess.CalledProcessError as e:
        print(f"转码失败: {e.stderr.decode()}")
        return False

# --- 使用示例 ---
if __name__ == "__main__":
    input_video = "/home/cat/uav_service/rtsp/1.mp4"  # 你的源文件
    convert_to_h264(input_video)