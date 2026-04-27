import sys
import types
import os
import time
import numpy as np
import open3d as o3d
from collections import deque

# --- 绕过 open3d.ml 冲突 ---
if 'open3d.ml' not in sys.modules:
    fake_ml = types.ModuleType('open3d.ml')
    sys.modules['open3d.ml'] = fake_ml

print("正在导入 Open3D...")
try:
    import open3d as o3d
    print(">> Open3D 导入成功！")
except Exception as e:
    print(f"!! 导入失败: {e}")
    sys.exit(1)

def capture_and_compress(input_hint, output_image):
    pcd_path = None
    
    # 1. 寻找文件逻辑
    if os.path.isfile(input_hint):
        pcd_path = input_hint
        print(f"找到指定文件: {pcd_path}")
    else:
        print(f"提示: 文件 '{input_hint}' 不存在，正在扫描当前目录...")
        files = [f for f in os.listdir('.') if f.lower().endswith('.pcd')]
        if not files:
            print("!! 错误：当前目录下没有找到任何 .pcd 文件。")
            return
        files.sort()
        pcd_path = os.path.abspath(files[0])
        print(f">> 自动选用文件: {pcd_path}")

    # 2. 读取点云
    print(f"正在读取: {pcd_path} ...")
    t_start = time.time()
    pcd = o3d.io.read_point_cloud(pcd_path)
    print(f"读取耗时: {time.time() - t_start:.2f}s")
    
    if pcd.is_empty():
        print("!! 警告：点云读取后为空。")
        return

    print(f"成功读取！点云包含 {len(pcd.points)} 个点。")


    # ----------------------
    # 4. 渲染部分
    # ----------------------
    print("正在计算最佳渲染参数...")
    
    bounds = pcd.get_axis_aligned_bounding_box()
    center = bounds.get_center()
    extent = bounds.get_extent()
    
    base_height = 1080
    x_span = extent[0]
    y_span = extent[1]
    
    if y_span == 0: y_span = 0.001
    if x_span == 0: x_span = 0.001
    
    aspect_ratio = x_span / y_span
    target_width = int(base_height * aspect_ratio)
    target_height = base_height
    
    min_dim = 640
    if target_width < min_dim:
        target_width = min_dim
        target_height = int(min_dim / aspect_ratio)
    if target_height < min_dim:
        target_height = min_dim
        target_width = int(min_dim * aspect_ratio)

    print(f">> 最终点云范围: X={x_span:.2f}, Y={y_span:.2f}")
    print(f">> 动态设定分辨率: {target_width} x {target_height}")

    renderer = o3d.visualization.rendering.OffscreenRenderer(target_width, target_height)
    renderer.scene.set_background([0, 0, 0, 1.0])
    
    mat = o3d.visualization.rendering.MaterialRecord()
    mat.shader = "defaultUnlit"
    mat.point_size = 4.0 
    
    renderer.scene.add_geometry("point_cloud", pcd, mat)
    
    max_span = max(x_span, y_span)
    camera_distance = max_span * 1.1
    
    eye = center + np.array([0, 0, camera_distance])
    look_at = center
    up = np.array([0, 1, 0])
    
    renderer.setup_camera(60.0, center, eye, up)

    print("正在渲染图像...")
    img = renderer.render_to_image()
    
    success = o3d.io.write_image(output_image, img)
    
    if success:
        print(f"✅ 截图成功已保存至: {output_image}")
        print(f"   图片尺寸: {target_width}x{target_height}")
    else:
        print("!! 保存图片失败。")
        
    del renderer 


if __name__ == "__main__":
    # 配置路径
    target_file = "/home/cat/slam_data/pcd/test.pcd" 
    output_img = "/home/cat/slam_data/pcd/map.png"
    
    # 压缩配置
    # 对于 900 万点，Level 12 可能会产生较多点，如果还觉得慢或文件大，可尝试 10 或 11
    compressed_pcd_output = "/home/panda/test/test_compressed.pcd"
    OCTREE_LEVEL = 9 
    
    os.environ['PYOPENGL_PLATFORM'] = 'egl'
    
    capture_and_compress(
        target_file, 
        output_img,
    )