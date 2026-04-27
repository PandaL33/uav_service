"""
公共认证工具模块
包含获取上传Token等通用的认证接口
"""
import json
import logging
import requests
from requests.auth import HTTPBasicAuth
from task_dispatcher.config import AUTH_CONFIG

logger = logging.getLogger('task_dispatcher.auth_manager')


class AuthManager:
    """
    认证管理器
    负责处理与认证、Token获取相关的逻辑
    """
    
    def __init__(self, server_url: str):
        """
        初始化认证管理器
        
        Args:
            server_url (str): 服务器基础地址 (例如: http://192.168.1.100:8080)
        """
        self.server_url = server_url.rstrip('/') # 确保URL末尾没有斜杠，避免拼接错误
        logger.info(f"AuthManager 初始化完成，服务器地址: {self.server_url}")

    def get_upload_token(self, file_properties: int = 0) -> str:
        """
        获取文件上传Token
        
        Args:
            file_properties (int, optional): 文件属性参数. 默认为 0.
            
        Returns:
            str: 成功返回Token字符串，失败返回空字符串
        """
        try:
            # 1. 构建URL (将参数作为查询参数拼接)
            # 使用 f-string 拼接，并动态传入 file_properties
            url = f"{self.server_url}{AUTH_CONFIG['TOKEN_ENDPOINT']}?fileProperties={file_properties}"
            
            username = AUTH_CONFIG['USERNAME']
            password = AUTH_CONFIG['PASSWORD']
            
            logger.info(f"正在请求获取上传Token，URL: {url}")
            
            # 2. 发送请求
            # verify=False 用于忽略SSL证书验证（适用于自签名证书）
            # timeout 设置为 5秒，防止永久阻塞
            response = requests.get(
                url, 
                auth=HTTPBasicAuth(username, password), 
                timeout=5, 
                verify=False # 警告：生产环境建议配置有效证书并设为 True
            )
            
            # 3. 处理响应
            # 使用 response.json() 直接解析，比 json.loads(response.text) 更健壮
            data = response.json()
            logger.info(f'获取Token响应数据: {data}')
            
            # 4. 结果校验
            if data.get('code') == 200:
                token = data.get('data', {}).get('uploadToken', '')
                if token:
                    logger.info("成功获取上传Token")
                    return token
                else:
                    logger.warning("响应Code为200，但Token字段为空")
            else:
                logger.error(f"获取Token失败，业务错误码: {data.get('code')}, 信息: {data.get('message')}")
                
            return ''
            
        except requests.exceptions.Timeout:
            logger.error("获取上传Token请求超时 (Timeout)")
            return ''
        except requests.exceptions.RequestException as e:
            logger.error(f"获取上传Token网络请求异常: {str(e)}")
            return ''
        except json.JSONDecodeError as e:
            logger.error(f"响应数据JSON解析失败: {str(e)}")
            return ''
        except Exception as e:
            logger.error(f"获取上传Token发生未知错误: {str(e)}")
            return ''