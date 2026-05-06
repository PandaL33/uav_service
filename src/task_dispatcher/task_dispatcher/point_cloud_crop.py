import open3d as o3d
import numpy as np
from collections import defaultdict

class PointCloudCrop:
    def projection_outlier_removal(self, pcd, plane='xy', grid_size=0.1, min_points_per_cell=3):
        """
        使用投影滤波提取“密集”区域的核心点云
        注意：此函数返回的是“保留下来的点”，而不是“被移除的噪点”。
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return o3d.geometry.PointCloud()
        
        # 选择投影维度
        if plane == 'xy':
            proj = points[:, :2]
        elif plane == 'xz':
            proj = points[:, [0, 2]]
        elif plane == 'yz':
            proj = points[:, 1:]
        else:
            raise ValueError("plane must be 'xy', 'xz', or 'yz'")
        
        # 计算网格索引
        # 防止除以零或网格过小导致索引溢出
        if grid_size <= 0: grid_size = 0.01
        indices = np.floor(proj / grid_size).astype(int)
        
        # 构建字典统计每个网格中的点索引
        cell_dict = defaultdict(list)
        for i, idx in enumerate(indices):
            key = tuple(idx)
            cell_dict[key].append(i)
            
        # 收集保留的点索引
        keep_indices = []
        for idx_list in cell_dict.values():
            if len(idx_list) >= min_points_per_cell:
                keep_indices.extend(idx_list)
        
        # 关键保护：如果没有点满足条件，返回空对象而不是崩溃
        if not keep_indices:
            return o3d.geometry.PointCloud()
                
        # 构造新点云
        filtered_pcd = o3d.geometry.PointCloud()
        filtered_pcd.points = o3d.utility.Vector3dVector(points[keep_indices])
        
        # 属性继承（颜色、法线）
        if pcd.has_colors():
            filtered_pcd.colors = o3d.utility.Vector3dVector(np.asarray(pcd.colors)[keep_indices])
        if pcd.has_normals():
            filtered_pcd.normals = o3d.utility.Vector3dVector(np.asarray(pcd.normals)[keep_indices])
            
        return filtered_pcd

    def auto_crop_pointcloud_area(self, filtered_xz, filtered_xy, filtered_yz, pcd):
        """
        根据三个投影面的有效范围，计算包围盒并裁剪原图
        """
        def safe_get_bounds(filtered_pcd):
            pts = np.asarray(filtered_pcd.points)
            if len(pts) == 0:
                return None
            return pts.min(axis=0), pts.max(axis=0)

        bounds_list = [
            safe_get_bounds(filtered_xz),
            safe_get_bounds(filtered_xy),
            safe_get_bounds(filtered_yz)
        ]
        
        # 过滤掉空的投影结果
        valid_bounds = [b for b in bounds_list if b is not None]
        
        if not valid_bounds:
            # 如果所有投影都是空的，说明全是噪点，返回空
            return o3d.geometry.PointCloud()

        # 合并所有有效投影的边界，取交集还是并集？
        # 这里取“最紧”的包围盒：
        # X范围取所有投影X的最小/最大值
        # Y范围取所有投影Y的最小/最大值
        # Z范围取所有投影Z的最小/最大值
        
        all_mins = np.array([b[0] for b in valid_bounds])
        all_maxs = np.array([b[1] for b in valid_bounds])
        
        # 使用最小值的最大值（最右/最前/最上）和最大值的最小值（最左/最后/最下）来取交集
        # 这样能保证裁剪框在所有投影的有效范围内
        x_min = np.max(all_mins[:, 0])
        x_max = np.min(all_maxs[:, 0])
        
        y_min = np.max(all_mins[:, 1])
        y_max = np.min(all_maxs[:, 1])
        
        z_min = np.max(all_mins[:, 2])
        z_max = np.min(all_maxs[:, 2])

        # 检查包围盒是否合法（防止出现 min > max 的情况）
        if x_min >= x_max or y_min >= y_max or z_min >= z_max:
             # 如果交集为空，退化为使用原始点云或者返回空，这里选择返回空
             return o3d.geometry.PointCloud()

        # 生成掩码
        points = np.asarray(pcd.points)
        mask = ((points[:, 0] >= x_min) & (points[:, 0] <= x_max) &
                (points[:, 1] >= y_min) & (points[:, 1] <= y_max) &
                (points[:, 2] >= z_min) & (points[:, 2] <= z_max))
        
        indices = np.where(mask)[0]
        if len(indices) == 0:
            return o3d.geometry.PointCloud()
            
        return pcd.select_by_index(indices)

    def auto_crop_pointcloud(self, pcd, x_ratio=0.05, y_ratio=0.05, z_ratio=0.1):
        """
        按比例向内收缩裁剪
        """
        points = np.asarray(pcd.points)
        if len(points) == 0:
            return pcd
            
        x_min, x_max = np.min(points[:, 0]), np.max(points[:, 0])
        y_min, y_max = np.min(points[:, 1]), np.max(points[:, 1])
        z_min, z_max = np.min(points[:, 2]), np.max(points[:, 2])
        
        # 计算边长
        x_span = x_max - x_min
        y_span = y_max - y_min
        z_span = z_max - z_min
        
        # 防止除以零（如果点云是一个平面或线）
        if x_span < 1e-6: x_span = 0
        if y_span < 1e-6: y_span = 0
        if z_span < 1e-6: z_span = 0

        # 向内收缩边界
        # 左边界向右移，右边界向左移
        crop_x_min = x_min + x_ratio * x_span
        crop_x_max = x_max - x_ratio * x_span
        
        crop_y_min = y_min + y_ratio * y_span
        crop_y_max = y_max - y_ratio * y_span
        
        # Z轴：只切顶部，保留底部
        crop_z_min = z_min
        crop_z_max = z_max - z_ratio * z_span
        
        # 确保边界合法
        if crop_x_min >= crop_x_max or crop_y_min >= crop_y_max or crop_z_min >= crop_z_max:
             return pcd # 裁剪过度，返回原数据或空

        mask = ((points[:, 0] >= crop_x_min) & (points[:, 0] <= crop_x_max) &
                (points[:, 1] >= crop_y_min) & (points[:, 1] <= crop_y_max) &
                (points[:, 2] >= crop_z_min) & (points[:, 2] <= crop_z_max))
        
        return pcd.select_by_index(np.where(mask)[0])

    def preprocess(self, pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
        # 1. 投影滤波：提取各个平面的“密集骨架”
        # 注意：这里的 min_points_per_cell 非常关键，设得太高会导致 filtered 为空
        filtered_xz = self.projection_outlier_removal(pcd, plane='xz', grid_size=0.1, min_points_per_cell=10)
        filtered_xy = self.projection_outlier_removal(pcd, plane='xy', grid_size=0.1, min_points_per_cell=10)
        filtered_yz = self.projection_outlier_removal(pcd, plane='yz', grid_size=0.1, min_points_per_cell=10)
        
        # 2. 根据骨架计算包围盒，裁剪原图
        # 这一步去除了外围的大片噪点
        cropped = self.auto_crop_pointcloud_area(filtered_xz, filtered_xy, filtered_yz, pcd)
        
        # 3. 最后微调：去除边界残留的噪点或顶部天空
        # x_ratio=0.02 表示左右各切掉 2%
        # z_ratio=0.35 表示切掉顶部 35% (去天)
        # final = self.auto_crop_pointcloud(cropped, x_ratio=0.02, y_ratio=0.02, z_ratio=0.35)
        
        return cropped