import numpy as np
import open3d as o3d
import time
from collections import deque
import logging

# 配置日志
logger = logging.getLogger('task_dispatcher.point_cloud_compress')

class PointCloudCompress:
    """
    基于八叉树原理的点云处理器类。
    提供统计滤波和自定义八叉树下采样功能。
    """

    def __init__(self, voxel_size=0.25, nb_neighbors=20, std_ratio=2.0):
        """
        初始化处理器
        :param voxel_size: 八叉树叶节点的最大边长 (体素大小)
        :param nb_neighbors: 统计滤波的邻域点数
        :param std_ratio: 统计滤波的标准差阈值
        """
        self.voxel_size = float(voxel_size)
        self.nb_neighbors = nb_neighbors
        self.std_ratio = std_ratio

    def process(pcd, voxel_size=0.25, filter_params={'nb_neighbors':20, 'std_ratio':2.0}):
        # 读取
        # 滤波（可选）
        #if filter_params:
        #    nb_neighbors = filter_params.get('nb_neighbors', 20)
        #    std_ratio = filter_params.get('std_ratio', 2.0)
        #    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
        #    print(f"滤波后点数: {len(pcd.points)}")

        # 体素下采样
        down_pcd = pcd.voxel_down_sample(voxel_size)
        # 滤波（可选）
        if filter_params:
            nb_neighbors = filter_params.get('nb_neighbors', 20)
            std_ratio = filter_params.get('std_ratio', 2.0)
            down_pcd, _ = down_pcd.remove_statistical_outlier(nb_neighbors=nb_neighbors, std_ratio=std_ratio)
            logger.info(f"滤波后点数: {len(pcd.points)}")

        logger.info(f"下采样后点数: {len(down_pcd.points)}")

        return down_pcd

# ==========================================
# 使用示例
# ==========================================
# if __name__ == "__main__":
#     # 模拟加载数据 (实际使用时请替换为 o3d.io.read_point_cloud)
#     # pcd = o3d.io.read_point_cloud("your_file.pcd")
    
#     # 创建一个测试点云 (随机生成 100 万个点)
#     logger.info("生成测试数据...")
#     pts = np.random.rand(1000000, 3) * 10
#     pcd = o3d.geometry.PointCloud()
#     pcd.points = o3d.utility.Vector3dVector(pts)
#     logger.info(f"测试点云点数: {len(pcd.points)}\n")

#     # 实例化处理器
#     # 设置体素大小为 0.5 (根据实际场景调整)
#     processor = PointCloudOctreeProcessor(voxel_size=0.5, nb_neighbors=20, std_ratio=2.0)

#     # 执行完整流程
#     result_pcd = processor.process(pcd, do_filter=True, do_downsample=True)

#     logger.info(f"\n最终结果点数: {len(result_pcd.points)}")
    
#     # 可视化 (如果需要)
#     # o3d.visualization.draw_geometries([result_pcd])