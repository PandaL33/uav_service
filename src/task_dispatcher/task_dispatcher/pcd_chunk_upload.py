#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
点云(.pcd)大文件分片上传工具

调用流程:
  1. 获取上传token     -> POST /provider/v1/file/getUploadTaskToken  得到 batchId
  2. 初始化分片任务     -> POST /file/upload/{batchId}/init
  3. 循环上传分片       -> POST /file/upload/{batchId}/chunk
  4. 合并分片           -> POST /file/merge/{batchId}                  返回 fileId
  5. 确认上传(持久化)   -> POST /provider/v1/file/uploadConfirm

依赖: pip install requests
"""

import os
import sys
import hashlib
import base64
import time
import argparse
import requests
import logging

# 配置日志
logger = logging.getLogger('task_dispatcher.pcd_chunk_upload')

class FileChunkUploader:
    """大文件分片上传客户端"""

    # provider接口前缀 (框架常量 Constant.PROVIDER_URI = "/provider/v1/")
    PROVIDER_PREFIX = "/provider/v1/"
    # 前端接口前缀
    PERSONAL_PREFIX = "/file"

    def __init__(self, base_url, service_user, service_password,
                 file_properties=0, fragment=1, chunk_size=5 * 1024 * 1024,
                 verify_ssl=False, timeout=300):
        """
        :param base_url:          文件服务地址，如 http://192.168.200.135:10103
        :param service_user:      服务认证用户名
        :param service_password:  服务认证明文密码
        :param file_properties:   0-公有文件, 1-私有文件
        :param fragment:          0-多文件上传, 1-大文件分片上传
        :param chunk_size:        分片大小(字节)，默认5MB
        :param verify_ssl:        是否校验SSL证书
        :param timeout:           HTTP超时秒数
        """
        self.base_url = base_url.rstrip("/")
        self.service_user = service_user
        self.service_password = service_password
        self.file_properties = file_properties
        self.fragment = fragment
        self.chunk_size = chunk_size
        self.verify_ssl = verify_ssl
        self.timeout = timeout
        self.session = requests.Session()

    # ------------------------------------------------------------------ #
    #  认证
    # ------------------------------------------------------------------ #
    def _gen_auth_header(self):
        """
        生成Authorization请求头
        格式: Basic Base64(serviceUser:md5(password+ts):ts)
        """
        ts = str(int(time.time() * 1000))
        md5_pwd = hashlib.md5((self.service_password + ts).encode("utf-8")).hexdigest()
        raw = "{}:{}:{}".format(self.service_user, md5_pwd, ts)
        auth_value = "Basic " + base64.b64encode(raw.encode("utf-8")).decode("utf-8")
        return {"Authorization": auth_value}

    # ------------------------------------------------------------------ #
    #  工具方法
    # ------------------------------------------------------------------ #
    @staticmethod
    def _check_result(resp, action):
        """检查响应结果，提取data"""
        if resp.status_code != 200:
            raise Exception("[{}] HTTP {}: {}".format(action, resp.status_code, resp.text))
        body = resp.json()
        # 兼容两种Result结构: {code:200, data:...} 或 {success:true, code:200, data:...}
        code = body.get("code")
        if code is not None and code != 200 and code != 0:
            raise Exception("[{}] 业务错误: {}".format(action, body))
        if "data" in body:
            return body["data"]
        return body

    @staticmethod
    def calculate_md5(file_path, chunk_size=8192):
        """分块计算文件MD5"""
        md5 = hashlib.md5()
        with open(file_path, "rb") as f:
            while True:
                buf = f.read(chunk_size)
                if not buf:
                    break
                md5.update(buf)
        return md5.hexdigest()

    # ------------------------------------------------------------------ #
    #  步骤1: 获取上传token
    # ------------------------------------------------------------------ #
    def get_upload_token(self):
        """
        GET /provider/v1/file/getUploadTaskToken
        :return: (batchId, uploadUrl)
        """
        url = "{}{}{}file/getUploadTaskToken".format(self.base_url, self.PERSONAL_PREFIX, self.PROVIDER_PREFIX)
        params = {
            "fileProperties": self.file_properties,
            "fragement": self.fragment,
        }
        headers = self._gen_auth_header()
        logger.info("[步骤1] 获取上传token...")
        resp = self.session.get(url, params=params, headers=headers,
                                verify=self.verify_ssl, timeout=self.timeout)
        data = self._check_result(resp, "获取上传token")
        batch_id = data.get("uploadToken")
        upload_url = data.get("uploadUrl")
        if not batch_id:
            raise Exception("获取batchId失败: {}".format(data))
        logger.info("  -> batchId = {}".format(batch_id))
        logger.info("  -> uploadUrl = {}".format(upload_url))
        return batch_id

    # ------------------------------------------------------------------ #
    #  步骤2: 初始化分片上传任务
    # ------------------------------------------------------------------ #
    def init_task(self, batch_id, identifier, total_size, chunk_size, file_name):
        """
        POST /file/upload/{batchId}/init
        :return: 任务信息
        """
        url = "{}{}/file/upload/{}/init".format(self.base_url, self.PERSONAL_PREFIX, batch_id)
        body = {
            "identifier": identifier,       # 文件MD5
            "totalSize": total_size,        # 文件总大小(字节)
            "chunkSize": chunk_size,        # 分片大小(字节)
            "fileName": file_name,          # 原始文件名
            "preSignedId": batch_id,        # 预签id，即batchId
        }
        logger.info("[步骤2] 初始化分片任务...")
        resp = self.session.post(url, json=body,
                                 verify=self.verify_ssl, timeout=self.timeout)
        data = self._check_result(resp, "初始化分片任务")
        finished = data.get("finished", False) if data else False
        logger.info("  -> 任务已创建, finished={}".format(finished))
        return data

    # ------------------------------------------------------------------ #
    #  步骤3: 上传单个分片
    # ------------------------------------------------------------------ #
    def upload_chunk(self, batch_id, chunk_data, chunk_number, total_chunks,
                     identifier, file_name):
        """
        POST /file/upload/{batchId}/chunk  (multipart/form-data)
        :return: 任务进度信息
        """
        url = "{}{}/file/upload/{}/chunk".format(self.base_url, self.PERSONAL_PREFIX, batch_id)
        files = {
            "file": (file_name, chunk_data, "application/octet-stream"),
        }
        data = {
            "chunkNumber": chunk_number,
            "totalChunks": total_chunks,
            "identifier": identifier,
            "fileName": file_name,
        }
        resp = self.session.post(url, files=files, data=data,
                                 verify=self.verify_ssl, timeout=self.timeout)
        result = self._check_result(resp, "上传分片[{}]".format(chunk_number))
        return result

    # ------------------------------------------------------------------ #
    #  步骤4: 合并分片
    # ------------------------------------------------------------------ #
    def merge(self, batch_id):
        """
        POST /file/merge/{batchId}
        :return: fileId (用于后续uploadConfirm)
        """
        url = "{}{}/file/merge/{}".format(self.base_url, self.PERSONAL_PREFIX, batch_id)
        logger.info("[步骤4] 合并分片...")
        resp = self.session.post(url,
                                 verify=self.verify_ssl, timeout=self.timeout)
        file_id = self._check_result(resp, "合并分片")
        logger.info("  -> fileId = {}".format(file_id))
        return file_id

    # ------------------------------------------------------------------ #
    #  步骤5: 确认上传(持久化文件)
    # ------------------------------------------------------------------ #
    def upload_confirm(self, batch_id, file_ids):
        """
        POST /provider/v1/file/uploadConfirm
        :return: FileInfo列表
        """
        url = "{}{}{}file/uploadConfirm".format(self.base_url, self.PERSONAL_PREFIX, self.PROVIDER_PREFIX)
        body = {
            "batchId": batch_id,
            "fileIds": file_ids,
        }
        headers = self._gen_auth_header()
        logger.info("[步骤5] 确认上传，持久化文件...")
        resp = self.session.post(url, json=body, headers=headers,
                                 verify=self.verify_ssl, timeout=self.timeout)
        data = self._check_result(resp, "确认上传")
        if data:
            for info in data:
                logger.info("  -> 文件ID: {}".format(info.get("id")))
                logger.info("  -> 文件名: {}".format(info.get("originalName")))
                logger.info("  -> 大小:   {}".format(info.get("size")))
                logger.info("  -> 下载URI: {}".format(info.get("downloadUri")))
        return data

    # ------------------------------------------------------------------ #
    #  完整上传流程
    # ------------------------------------------------------------------ #
    def upload_file(self, file_path):
        """
        完整的分片上传流程
        :param file_path: 本地文件路径
        :return: fileId
        """
        # file_name = os.path.basename(file_path)
        timestamp = int(time.time() * 1000)
        file_name = f'point_cloud_{timestamp}.pcd'
        total_size = os.path.getsize(file_path)
        chunk_size = self.chunk_size

        # 计算分片数量
        total_chunks = total_size // chunk_size
        if total_size % chunk_size != 0:
            total_chunks += 1

        logger.info("文件: {}".format(file_name))
        logger.info("大小: {} 字节 ({:.2f} MB)".format(total_size, total_size / 1024 / 1024))
        logger.info("分片: {} 片, 每片 {} 字节 ({:.2f} MB)".format(
            total_chunks, chunk_size, chunk_size / 1024 / 1024))

        # 计算文件MD5
        logger.info("正在计算文件MD5...")
        identifier = self.calculate_md5(file_path)
        logger.info("  -> MD5 = {}".format(identifier))

        # 步骤1: 获取上传token
        batch_id = self.get_upload_token()

        # 步骤2: 初始化任务
        self.init_task(batch_id, identifier, total_size, chunk_size, file_name)

        # 步骤3: 循环上传分片
        logger.info("[步骤3] 开始上传分片 (共{}片)...".format(total_chunks))
        with open(file_path, "rb") as f:
            for i in range(1, total_chunks + 1):
                chunk_data = f.read(chunk_size)
                self.upload_chunk(batch_id, chunk_data, i, total_chunks,
                                  identifier, file_name)
                # 进度显示
                percent = i * 100 // total_chunks
                logger.info("  -> 分片 {}/{} 上传完成 [{}%]".format(i, total_chunks, percent))
        logger.info("  -> 所有分片上传完成")

        # 步骤4: 合并分片
        file_id = self.merge(batch_id)

        # 步骤5: 确认上传
        #self.upload_confirm(batch_id, [file_id])

        logger.info("上传成功! fileId = {}".format(file_id))
        return file_id
