#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
服务管理模块，用于获取系统服务列表和控制服务状态
"""

import subprocess
import re
import logging
from typing import List, Dict, Any, Optional
from config import ROBOT_TYPE

logger = logging.getLogger(__name__)

# 自定义异常类
class ServiceManagerError(Exception):
    """服务管理相关的基础异常类"""
    pass


class ServiceControlError(ServiceManagerError):
    """服务控制异常"""
    pass


class ServiceListError(ServiceManagerError):
    """获取服务列表异常"""
    pass

if ROBOT_TYPE == "uav":
    # 服务映射配置
    SERVICE_CONFIG = {
        'cam_control': '云台控制服务',
        'microxrce-agent': '飞控服务',
        'fast_lio': '点云服务',
        'task_dispatcher': '调度服务',
        'lio_sam_nav2': '导航服务',
        'auto_buildmap': '建图服务'
    }
else:
    # 服务映射配置
    SERVICE_CONFIG = {
        'robot_control': '机器控制服务',
        'task_dispatcher': '调度服务',
        'nav2': '导航服务',
        'auto_buildmap': '建图服务'
    }


def get_service_list() -> Dict[str, Any]:
    """
    获取系统服务列表和状态
    
    Returns:
        Dict: 包含服务列表信息的字典，格式符合接口文档要求
    
    Raises:
        ServiceListError: 获取服务列表失败时抛出
    """
    logger.debug("开始获取服务列表")
    
    try:
        # 使用systemctl命令获取所有服务状态
        logger.debug("执行systemctl list-units命令获取服务列表")
        result = subprocess.run(['systemctl', 'list-units', '--type=service', '--all'], 
                               capture_output=True, text=True, check=True, timeout=10)
        
        services = []
        # 解析输出获取服务状态
        for service_id, service_name in SERVICE_CONFIG.items():
            logger.debug(f"检查服务: {service_id}")
            # 查找特定服务的状态
            service_pattern = rf'{service_id}[\.\s]'
            match = re.search(service_pattern, result.stdout)
            
            if match:
                # 获取服务的详细状态
                try:
                    status_result = subprocess.run(['systemctl', 'is-active', f'{service_id}'],
                                                 capture_output=True, text=True, timeout=5)
                    status = 1 if status_result.stdout.strip() == 'active' else 0
                    logger.debug(f"服务 {service_id} 状态: {'active' if status else 'inactive'}")
                except subprocess.TimeoutExpired:
                    logger.warning(f"获取服务 {service_id} 状态超时，默认为停止状态")
                    status = 0
                except Exception as e:
                    logger.error(f"获取服务 {service_id} 状态失败: {str(e)}")
                    status = 0
            else:
                # 服务不存在或未安装
                # logger.debug(f"服务 {service_id} 不存在或未安装")
                status = 0
            
            services.append({
                'serviceId': service_id,
                'name': service_name,
                'status': status
            })
        
        # logger.info(f"成功获取服务列表，共{len(services)}个服务")
        logger.debug(f"服务列表详情: {services}")
        return services
        
    except subprocess.CalledProcessError as e:
        error_msg = f"执行systemctl命令失败: {e}, 错误输出: {e.stderr}"
        logger.error(error_msg)
        # 返回默认服务列表，全部为停止状态
        services = [{
            'serviceId': service_id,
            'name': service_name,
            'status': 0
        } for service_id, service_name in SERVICE_CONFIG.items()]
        logger.warning(f"返回默认服务列表（全部停止状态）")
        return services
    except subprocess.TimeoutExpired:
        error_msg = "执行systemctl命令超时"
        logger.error(error_msg)
        # 返回默认服务列表，全部为停止状态
        services = [{
            'serviceId': service_id,
            'name': service_name,
            'status': 0
        } for service_id, service_name in SERVICE_CONFIG.items()]
        logger.warning(f"返回默认服务列表（全部停止状态）")
        return services
    except Exception as e:
        error_msg = f"获取服务列表失败: {str(e)}"
        logger.error(error_msg)
        # 返回默认服务列表，全部为停止状态
        services = [{
            'serviceId': service_id,
            'name': service_name,
            'status': 0
        } for service_id, service_name in SERVICE_CONFIG.items()]
        logger.warning(f"返回默认服务列表（全部停止状态）")
        return services


def service_control(services: List[Dict[str, str]]) -> Dict[str, Any]:
    """
    控制服务状态（启动、停止、重启）
    
    Args:
        services: 服务控制列表，每个元素包含serviceId和control字段
                  control取值: stop-停止/start-启动/restart-重启
    
    Returns:
        Dict: 操作后的服务列表状态，格式符合接口文档要求
    
    Raises:
        ServiceControlError: 服务控制过程中出现严重错误时抛出
    """
    logger.debug(f"开始服务控制操作，共{len(services)}个服务")
    valid_controls = ['stop', 'start', 'restart']
    
    for index, service in enumerate(services):
        logger.debug(f"处理服务控制请求 #{index + 1}: {service}")
        service_id = service.get('serviceId')
        control = service.get('control')
        
        if not service_id:
            logger.warning(f"服务ID为空，跳过控制操作")
            continue
        
        if not control or control not in valid_controls:
            logger.warning(f"无效的控制命令: {control}，有效值为: {valid_controls}，跳过服务: {service_id}")
            continue
        
        if service_id not in SERVICE_CONFIG:
            logger.warning(f"未知的服务ID: {service_id}，支持的服务ID: {list(SERVICE_CONFIG.keys())}，跳过控制操作")
            continue
        
        try:
            # 检查服务是否存在
            check_result = subprocess.run(
                ['systemctl', 'is-enabled', service_id],
                capture_output=True,
                text=True
            )
            
            if check_result.returncode != 0 and 'does not exist' in check_result.stderr:
                logger.warning(f"服务 {service_id} 不存在，无法执行 {control} 操作")
                continue
            
            # 执行服务控制命令，需要sudo权限
            logger.info(f"执行服务控制: {service_id} {control}")
            result = subprocess.run(
                ['sudo', 'systemctl', control, service_id],
                capture_output=True,
                text=True,
                timeout=10  # 设置超时时间，防止命令长时间阻塞
            )
            
            if result.returncode in [0, -15, -9]:  # 0:成功, -15:正常终止, -9:强制终止
                logger.info(f"成功控制服务: {service_id} {control}")
                # 验证操作结果
                verify_result = subprocess.run(
                    ['systemctl', 'is-active', service_id],
                    capture_output=True,
                    text=True
                )
                current_status = verify_result.stdout.strip()
                logger.debug(f"服务 {service_id} 当前状态: {current_status}")
            else:
                error_msg = f"控制服务失败: {service_id} {control}, 返回码: {result.returncode}, 错误: {result.stderr}"
                logger.error(error_msg)
                # 不抛出异常，继续处理其他服务
                
        except subprocess.TimeoutExpired:
            error_msg = f"执行服务控制超时: {service_id} {control}（超过30秒）"
            logger.error(error_msg)
        except PermissionError:
            error_msg = f"权限不足，无法控制服务: {service_id} {control}（需要sudo权限）"
            logger.error(error_msg)
        except Exception as e:
            error_msg = f"执行服务控制时出错: {service_id} {control}, 错误类型: {type(e).__name__}, 错误信息: {str(e)}"
            logger.error(error_msg)
    
    logger.debug("服务控制操作完成，获取最新服务状态")
    # 返回操作后的服务列表状态
    return get_service_list()


def handle_service_list_request() -> Dict[str, Any]:
    """
    处理服务列表查询请求
    
    Returns:
        Dict: 服务列表响应数据
    """
    logger.debug("收到服务列表查询请求")
    try:
        result = get_service_list()
        logger.debug(f"服务列表查询请求处理完成，返回{len(result)}个服务")
        return result
    except Exception as e:
        logger.error(f"处理服务列表查询请求失败: {str(e)}")
        # 即使发生异常也返回默认服务列表
        services = [{
            'serviceId': service_id,
            'name': service_name,
            'status': 0
        } for service_id, service_name in SERVICE_CONFIG.items()]
        return services


def handle_service_control_request(services: List[Dict[str, str]]) -> Dict[str, Any]:
    """
    处理服务控制请求
    
    Args:
        services: 服务控制列表
    
    Returns:
        Dict: 服务控制响应数据
    """
    logger.info(f"收到服务控制请求，包含{len(services)}个服务控制命令")
    try:
        # 参数验证
        if not isinstance(services, list):
            raise ValueError("services参数必须是列表类型")
        
        result = service_control(services)
        logger.info("服务控制请求处理完成")
        return result
    except ValueError as e:
        logger.error(f"服务控制请求参数错误: {str(e)}")
        # 返回当前服务状态
        return get_service_list()
    except Exception as e:
        logger.error(f"处理服务控制请求时发生错误: {str(e)}")
        # 即使发生异常也返回当前服务状态
        return get_service_list()


if __name__ == '__main__':
    # 测试获取服务列表功能
    print("===== 测试获取服务列表 =====")
    services_info = get_service_list()
    print("服务列表:")
    for service in services_info:
        print(f"ID: {service['serviceId']}, 名称: {service['name']}, 状态: {'运行中' if service['status'] else '已停止'}")
    
    # 测试服务控制功能（需要sudo权限）
    print("\n===== 测试服务控制功能（需要sudo权限）=====")
    print("注意：以下测试需要在有权限的环境中运行")
    # 示例控制命令，仅作演示
    test_controls = [
        {'serviceId': 'task_dis', 'control': 'status'}  # 仅查询状态，不实际控制
    ]
    print(f"测试控制命令: {test_controls}")