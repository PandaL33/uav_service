#!/usr/bin/env python3
"""
点云ICP精准配准模块 (CloudCompare Fine Registration 风格)

该模块实现了点云的ICP（Iterative Closest Point）配准算法，
参考 CloudCompare 中的 Fine Registration 工具实现。

核心参数:
  - RMS difference: 相邻迭代RMS变化量收敛阈值 (默认: 1.0e-5)
  - Max iterations: 最大迭代次数 (默认: 20)
  - Overlap: 重叠率, 用于筛选对应点 (默认: 100%)
  - Point-to-plane: 使用点到面距离代替点到点 (默认: 开启)
  - Random sampling limit: 随机采样上限 (默认: 50000)
"""

import numpy as np
import open3d as o3d
from typing import Tuple, Optional
import logging
import os
import time

logger = logging.getLogger('cloud_compare_icp')

class CloudCompareIcp:
    def __init__(self):
        """初始化ICP配准类"""
        pass

    
    def _compute_default_distance(self, aligned_cloud: np.ndarray, reference_cloud: np.ndarray) -> float:
        """根据点云包围盒对角线自动计算默认对应点距离阈值 (取对角线长度的1%)"""
        combined = np.vstack([aligned_cloud, reference_cloud])
        bbox_min = combined.min(axis=0)
        bbox_max = combined.max(axis=0)
        diagonal = np.linalg.norm(bbox_max - bbox_min)
        return max(diagonal * 0.01, 1e-6)

    def cloud_compare_icp(
        self,
        aligned_cloud: np.ndarray,
        reference_cloud: np.ndarray,
        max_iterations: int = 20,
        rms_diff: float = 1e-5,
        max_correspondence_distance: Optional[float] = None,
        overlap: float = 1.0,
        use_point_to_plane: bool = True,
        random_sample_limit: int = 50000,
        verbose: bool = True,
    ) -> Tuple[np.ndarray, np.ndarray, float, int]:
        """
        CloudCompare 风格的 Fine Registration (ICP) 配准

        参数:
            aligned_cloud: 待配准的点云，形状为 (N, 3)
            reference_cloud: 参考点云，形状为 (M, 3)
            max_iterations: 最大迭代次数 (默认: 20)
            rms_diff: RMS difference 收敛阈值 (默认: 1.0e-5)
            max_correspondence_distance: 对应点最大距离 (默认: 自动计算)
            overlap: 重叠率, 0~1 之间 (默认: 1.0)
            use_point_to_plane: 是否使用点到面模式 (默认: True)
            random_sample_limit: 随机采样上限 (默认: 50000)
            verbose: 是否输出迭代信息

        返回:
            transformed_aligned: 配准变换后的点云 (与原始 aligned_cloud 同尺寸)
            transformation: 累积变换矩阵 (4x4)
            fitness: 配准适应度
            iterations: 实际迭代次数
        """
        source = aligned_cloud.copy()
        target = reference_cloud.copy()

        # 自动计算对应点距离阈值
        if max_correspondence_distance is None:
            max_correspondence_distance = self._compute_default_distance(source, target)
            if verbose:
                logger.info(f"  自动计算对应点距离阈值: {max_correspondence_distance:.6f}")

        # 随机采样（如果点云过大）
        if len(source) > random_sample_limit:
            if verbose:
                logger.info(f"  待配准点云超过限制 ({len(source)} > {random_sample_limit}), 随机采样")
            indices = np.random.choice(len(source), random_sample_limit, replace=False)
            source = source[indices]
        if len(target) > random_sample_limit:
            if verbose:
                logger.info(f"  参考点云超过限制 ({len(target)} > {random_sample_limit}), 随机采样")
            indices = np.random.choice(len(target), random_sample_limit, replace=False)
            target = target[indices]

        # 构建目标点云的 KDTree
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(target)
        target_tree = o3d.geometry.KDTreeFlann(target_pcd)

        # 点到面模式: 估计目标点云法向量
        target_normals = None
        if use_point_to_plane:
            target_pcd.estimate_normals(
                search_param=o3d.geometry.KDTreeSearchParamKNN(knn=20))
            target_normals = np.asarray(target_pcd.normals)

        total_transform = np.eye(4)
        prev_rms = float('inf')
        actual_iterations = 0
        final_src_corr_count = 0
        max_sq_dist = max_correspondence_distance ** 2

        for iteration in range(max_iterations):
            # Step 1: 寻找最近邻对应点对
            src_corr = []
            tgt_corr = []
            sq_dists = []
            tgt_indices = []

            for i in range(len(source)):
                point = source[i]
                [k, idx, dist] = target_tree.search_knn_vector_3d(point, 1)
                if k > 0 and dist[0] < max_sq_dist:
                    src_corr.append(point)
                    tgt_corr.append(target[idx[0]])
                    sq_dists.append(dist[0])
                    tgt_indices.append(idx[0])

            final_src_corr_count = len(src_corr)

            if final_src_corr_count < 3:
                if verbose:
                    logger.info(f"  迭代 {iteration + 1}: 对应点不足 ({final_src_corr_count}), 停止配准")
                break

            src_corr = np.array(src_corr)
            tgt_corr = np.array(tgt_corr)
            sq_dists = np.array(sq_dists)
            tgt_indices = np.array(tgt_indices)

            # Step 2: 重叠率过滤 — 按距离排序保留最佳 overlap 比例
            if overlap < 1.0:
                n_keep = max(int(final_src_corr_count * overlap), 3)
                order = np.argsort(sq_dists)
                keep = order[:n_keep]
                src_corr = src_corr[keep]
                tgt_corr = tgt_corr[keep]
                tgt_indices = tgt_indices[keep]
                sq_dists = sq_dists[keep]

            # Step 3: 计算变换矩阵
            if use_point_to_plane and target_normals is not None:
                # 点到面: 使用 Open3D 的线性化最小二乘
                src_pcd = o3d.geometry.PointCloud()
                src_pcd.points = o3d.utility.Vector3dVector(src_corr)
                tgt_pcd = o3d.geometry.PointCloud()
                tgt_pcd.points = o3d.utility.Vector3dVector(tgt_corr)
                corr_normals = target_normals[tgt_indices]
                tgt_pcd.normals = o3d.utility.Vector3dVector(corr_normals)

                n_corr = len(src_corr)
                corres = o3d.utility.Vector2iVector(
                    np.column_stack([np.arange(n_corr), np.arange(n_corr)]))
                estimation = o3d.pipelines.registration.TransformationEstimationPointToPlane()
                iter_transform = estimation.compute_transformation(src_pcd, tgt_pcd, corres)
            else:
                # 点到点: SVD 分解求解刚性变换
                src_centroid = np.mean(src_corr, axis=0)
                tgt_centroid = np.mean(tgt_corr, axis=0)
                src_centered = src_corr - src_centroid
                tgt_centered = tgt_corr - tgt_centroid
                H = src_centered.T @ tgt_centered
                U, _, Vt = np.linalg.svd(H)
                R = Vt.T @ U.T
                if np.linalg.det(R) < 0:
                    Vt[-1, :] *= -1
                    R = Vt.T @ U.T
                t_vec = tgt_centroid - R @ src_centroid
                iter_transform = np.eye(4)
                iter_transform[:3, :3] = R
                iter_transform[:3, 3] = t_vec

            # Step 4: 变换整个源点云
            source = (iter_transform[:3, :3] @ source.T).T + iter_transform[:3, 3]
            total_transform = iter_transform @ total_transform

            # Step 5: 计算当前 RMS 误差
            current_rms = np.sqrt(np.mean(sq_dists))

            # Step 6: 检查 RMS difference 收敛
            rms_change = abs(prev_rms - current_rms)
            actual_iterations = iteration + 1

            if verbose:
                logger.info(f"  迭代 {actual_iterations:3d}: RMS = {current_rms:.8f}, "
                      f"ΔRMS = {rms_change:.6e}, 对应点 = {final_src_corr_count}")

            if rms_change < rms_diff and iteration > 0:
                if verbose:
                    logger.info(f"  收敛: ΔRMS ({rms_change:.6e}) < RMS difference ({rms_diff})")
                break

            prev_rms = current_rms

        # 适应度: 最终有对应关系的点占源点云的比例
        total_source_count = len(source)
        fitness = final_src_corr_count / total_source_count if total_source_count > 0 else 0.0

        return source, total_transform, fitness, actual_iterations

    def icp_registration(self, aligned_cloud: np.ndarray,
                         reference_cloud: np.ndarray,
                         threshold: float = 0.02) -> Tuple[np.ndarray, np.ndarray, float]:
        """
        (兼容接口) 使用 CloudCompare 风格 ICP 配准，默认一次迭代模式

        参数:
            aligned_cloud: 待配准的点云，形状为 (N, 3)
            reference_cloud: 参考点云，形状为 (M, 3)
            threshold: 对应点距离阈值

        返回:
            transformed_aligned: 配准变换后的点云
            transformation: 变换矩阵 (4x4)
            fitness: 配准适应度
        """
        result, transform, fitness, _ = self.cloud_compare_icp(
            aligned_cloud, reference_cloud,
            max_iterations=1,
            rms_diff=1e-5,
            max_correspondence_distance=threshold,
            overlap=1.0,
            use_point_to_plane=True,
            random_sample_limit=50000,
            verbose=False,
        )
        return result, transform, fitness

    
    def find_correspondences(self, source_points: np.ndarray, target_points: np.ndarray, threshold: float = 0.02):
        """为源点云中的每个点在目标点云中寻找最近邻点"""
        target_pcd = o3d.geometry.PointCloud()
        target_pcd.points = o3d.utility.Vector3dVector(target_points)
        kdtree = o3d.geometry.KDTreeFlann(target_pcd)

        correspondences = []
        for i in range(len(source_points)):
            point = source_points[i]
            [k, idx, _] = kdtree.search_knn_vector_3d(point, 1)
            if idx and _[0] < threshold:
                correspondences.append((i, idx[0]))
        return correspondences

    
    def compute_rms_difference_transform(self, source_points: np.ndarray, target_points: np.ndarray,
                                          correspondences: list) -> Tuple[np.ndarray, float]:
        """基于对应点对和RMS差异计算最优变换矩阵"""
        if len(correspondences) < 3:
            raise ValueError("至少需要3对对应点来计算变换")

        src_corr = np.array([source_points[i] for i, j in correspondences])
        tgt_corr = np.array([target_points[j] for i, j in correspondences])

        src_centroid = np.mean(src_corr, axis=0)
        tgt_centroid = np.mean(tgt_corr, axis=0)
        src_centered = src_corr - src_centroid
        tgt_centered = tgt_corr - tgt_centroid
        H = src_centered.T @ tgt_centered
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        t = tgt_centroid - R @ src_centroid

        transform_matrix = np.eye(4)
        transform_matrix[:3, :3] = R
        transform_matrix[:3, 3] = t

        transformed_src = (R @ src_corr.T).T + t
        rms_error = np.sqrt(np.mean(np.sum((transformed_src - tgt_corr) ** 2, axis=1)))
        return transform_matrix, rms_error

    
    def rigid_transform_3D(self, A: np.ndarray, B: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """使用SVD计算刚性变换矩阵"""
        assert A.shape == B.shape
        num_rows, num_cols = A.shape
        if num_rows != 3:
            raise Exception(f"matrix A is not 3xN, it is {num_rows}x{num_cols}")

        centroid_A = np.mean(A, axis=1).reshape(-1, 1)
        centroid_B = np.mean(B, axis=1).reshape(-1, 1)
        Am = A - centroid_A
        Bm = B - centroid_B
        H = Am @ Bm.T
        U, S, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[2, :] *= -1
            R = Vt.T @ U.T
        t = -R @ centroid_A + centroid_B
        t = t.ravel()
        return R, t

    
    def load_point_cloud(self, file_path: str) -> np.ndarray:
        """加载点云文件"""
        if not os.path.exists(file_path):
            raise FileNotFoundError(f"点云文件不存在: {file_path}")
        pcd = o3d.io.read_point_cloud(file_path)
        if len(pcd.points) == 0:
            raise ValueError(f"点云文件为空: {file_path}")
        return np.asarray(pcd.points)

    
    def save_point_cloud(self, file_path: str, points: np.ndarray) -> None:
        """保存点云数据到文件"""
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(file_path, pcd)
        logger.info(f"点云已保存至: {file_path}")

    def registrator(self, args):
        """
        封装原main函数逻辑，支持传参调用
        :param args: argparse解析后的参数对象 / 字典
        """
        # 兼容字典传参
        if isinstance(args, dict):
            from argparse import Namespace
            args = Namespace(**args)
            
        start_total_time = time.time()

        try:
            logger.info(f"加载待配准点云: {args.aligned}")
            aligned_cloud = self.load_point_cloud(args.aligned)

            logger.info(f"加载参考点云: {args.reference}")
            reference_cloud = self.load_point_cloud(args.reference)

            logger.info(f"待配准点云形状: {aligned_cloud.shape}")
            logger.info(f"参考点云形状: {reference_cloud.shape}")

            # 处理对应点距离
            corr_dist = args.correspondence_distance
            if corr_dist is None and args.threshold is not None:
                corr_dist = args.threshold

            # 打印参数
            mode = "点到面 (Point-to-Plane)" if args.point_to_plane else "点到点 (Point-to-Point)"
            logger.info(f"\n执行 CloudCompare 风格 ICP 配准...")
            logger.info(f"  模式: {mode}")
            logger.info(f"  最大迭代次数: {args.max_iterations}")
            logger.info(f"  RMS difference: {args.rms_diff}")
            logger.info(f"  重叠率: {args.overlap}")
            logger.info(f"  随机采样限制: {args.random_sample_limit}")
            if corr_dist is not None:
                logger.info(f"  对应点距离阈值: {corr_dist}")

            # 执行配准
            transformed_points, transform_matrix, fitness, iterations = self.cloud_compare_icp(
                aligned_cloud, reference_cloud,
                max_iterations=args.max_iterations,
                rms_diff=args.rms_diff,
                max_correspondence_distance=corr_dist,
                overlap=args.overlap,
                use_point_to_plane=args.point_to_plane,
                random_sample_limit=args.random_sample_limit,
                verbose=True,
            )

            logger.info(f"\n配准完成!")
            logger.info(f"  实际迭代次数: {iterations}")
            logger.info(f"  配准适应度: {fitness:.6f}")
            logger.info(f"  变换矩阵:\n{transform_matrix}")

            # 计算配准误差
            if len(transformed_points) == len(reference_cloud):
                error = np.mean(np.linalg.norm(transformed_points - reference_cloud, axis=1))
                logger.info(f"  平均配准误差: {error:.6f}m")
            else:
                # 用最近邻方式计算近似误差
                ref_pcd = o3d.geometry.PointCloud()
                ref_pcd.points = o3d.utility.Vector3dVector(reference_cloud)
                ref_tree = o3d.geometry.KDTreeFlann(ref_pcd)
                nn_errors = []
                for pt in transformed_points:
                    [k, idx, dist] = ref_tree.search_knn_vector_3d(pt, 1)
                    if k > 0:
                        nn_errors.append(np.sqrt(dist[0]))
                if nn_errors:
                    avg_error = np.mean(nn_errors)
                    logger.info(f"  平均最近邻配准误差: {avg_error:.6f}m")

            # 输出文件路径
            if args.output:
                output_path = args.output
            else:
                base_name = os.path.splitext(os.path.basename(args.aligned))[0]
                dir_name = os.path.dirname(args.aligned)
                output_path = os.path.join(dir_name, f"{base_name}_registered.pcd")

            output_dir = os.path.dirname(output_path)
            if output_dir and not os.path.exists(output_dir):
                os.makedirs(output_dir)
            
            # 将变换矩阵应用到原始点云上
            rotation = transform_matrix[:3, :3]
            translation = transform_matrix[:3, 3]
            transformed_aligned_cloud = (rotation @ aligned_cloud.T).T + translation
            logger.info(f"配准点云形状: {transformed_aligned_cloud.shape}")
            
            # 保存变换后的点云
            self.save_point_cloud(output_path, transformed_aligned_cloud)

            total_time = time.time() - start_total_time
            logger.info(f"总耗时: {total_time:.4f}秒")
            
            return transform_matrix, fitness, iterations

        except FileNotFoundError as e:
            logger.error(f"错误: {e}")
        except ValueError as e:
            logger.error(f"错误: {e}")
        except Exception as e:
            logger.error(f"发生未知错误: {e}")

    def run_registrator(self, aligned_file, reference_file, output_file):
        """
        执行主函数
        :param args: 参数对象
        :return:
        """
        params = {
            "aligned": aligned_file,
            "reference": reference_file,
            "output": output_file,
            "rms_diff": 1e-5,
            "max_iterations": 20,
            "overlap": 1.0,
            "point_to_plane": True,
            "random_sample_limit": 50000,
            "correspondence_distance": None,
            "threshold": None
        }
        
        return self.registrator(params)
    
if __name__ == "__main__":
    reference_file = "../3dmap.pcd"
    aligned_file = "../point_cloud_1778656628539.pcd"

    
    # 实例化类并运行
    icp = CloudCompareIcp()
    icp.run_registrator(aligned_file, reference_file, "r.pcd")

