#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
服务客户端惰性注册工具。

rclpy 单线程 executor 每处理一条消息，都要用节点上的全部实体（订阅、定时器、
服务客户端、QoS 事件）重建一次 WaitSet。空闲的服务客户端不产生任何消息，却会
永久计入每轮重建开销；在方法里反复 create_client 更是只增不减地泄漏实体。

用法（宿主类需持有 self.node）：

    class Foo:
        algo_client = lazy_client(AlgoControl, '/algo_control')

首次访问时才真正创建，之后复用同一个客户端。
"""


def lazy_client(msg_type, service_name, attr=None):
    storage = attr or '_client_' + service_name.strip('/').replace('/', '_')

    def _getter(self):
        client = self.__dict__.get(storage)
        if client is None:
            client = self.node.create_client(msg_type, service_name)
            self.__dict__[storage] = client
        return client

    return property(_getter)
