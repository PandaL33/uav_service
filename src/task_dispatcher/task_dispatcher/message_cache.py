#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
消息缓存管理器
用于缓存主动上报的消息，支持10秒过期机制
"""
import time
from typing import Dict, Any, Optional


class MessageCacheManager:
    """消息缓存管理器类"""
    
    def __init__(self, cache_time_seconds: int = 10):
        """
        初始化消息缓存管理器
        
        Args:
            cache_time_seconds: 缓存时间（秒），默认10秒
        """
        self.cache_time_seconds = cache_time_seconds
        # 缓存结构: {seq: {"message": 原始消息, "timestamp": 缓存时间戳}}
        self._cache: Dict[str, Dict[str, Any]] = {}
    
    def add_message(self, seq: str, message: Dict[str, Any]) -> None:
        """
        添加消息到缓存
        
        Args:
            seq: 消息序列号
            message: 完整的上报消息
        """
        self._cache[seq] = {
            "message": message,
            "timestamp": time.time()
        }
        # 清理过期缓存
        self._clean_expired_cache()
    
    def get_message_by_seq(self, seq: str) -> Optional[Dict[str, Any]]:
        """
        根据序列号获取缓存的消息
        
        Args:
            seq: 消息序列号
            
        Returns:
            缓存的消息字典，如果不存在或已过期则返回None
        """
        # 先清理过期缓存
        self._clean_expired_cache()
        
        # 查找消息
        if seq in self._cache:
            return self._cache[seq]["message"]
        return None
    
    def _clean_expired_cache(self) -> None:
        """
        清理过期的缓存消息
        """
        current_time = time.time()
        expired_seqs = []
        
        # 找出所有过期的seq
        for seq, cached_data in self._cache.items():
            if current_time - cached_data["timestamp"] > self.cache_time_seconds:
                expired_seqs.append(seq)
        
        # 删除过期的缓存
        for seq in expired_seqs:
            del self._cache[seq]
    
    def get_cache_size(self) -> int:
        """
        获取当前缓存的大小
        
        Returns:
            缓存的消息数量
        """
        self._clean_expired_cache()
        return len(self._cache)


# 创建全局缓存管理器实例
message_cache_manager = MessageCacheManager()