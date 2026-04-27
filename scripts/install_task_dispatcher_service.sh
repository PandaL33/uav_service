#!/bin/bash

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}===== ROS2 任务派发器服务安装脚本 =====${NC}"

# 检查是否以root权限运行
if [ "$(id -u)" != "0" ]; then
   echo -e "${RED}错误: 此脚本需要以root权限运行${NC}"
   echo -e "请使用 sudo 命令执行此脚本"
   exit 1
fi

# 检测系统架构
ARCH=$(uname -m)
echo -e "${YELLOW}检测到系统架构: $ARCH${NC}"

# 服务文件路径
ARM_SERVICE="scripts/task_dispatcher_arm.service"
X86_SERVICE="scripts/task_dispatcher_x86.service"
SERVICE_NAME="task_dispatcher.service"
SYSTEMD_PATH="/etc/systemd/system/$SERVICE_NAME"

# 根据架构选择合适的服务文件
if [ "$ARCH" = "x86_64" ]; then
    if [ -f "$X86_SERVICE" ]; then
        echo -e "${GREEN}使用x86_64架构服务文件${NC}"
        cp "$X86_SERVICE" "$SYSTEMD_PATH"
        echo -e "${GREEN}服务文件已复制到 $SYSTEMD_PATH${NC}"
    else
        echo -e "${RED}错误: 找不到x86_64服务文件 $X86_SERVICE${NC}"
        exit 1
    fi
elif [ "$ARCH" = "aarch64" ]; then
    if [ -f "$ARM_SERVICE" ]; then
        echo -e "${GREEN}使用ARM架构服务文件${NC}"
        cp "$ARM_SERVICE" "$SYSTEMD_PATH"
        echo -e "${GREEN}服务文件已复制到 $SYSTEMD_PATH${NC}"
    else
        echo -e "${RED}错误: 找不到ARM服务文件 $ARM_SERVICE${NC}"
        exit 1
    fi
else
    echo -e "${YELLOW}警告: 未知架构 $ARCH${NC}"
    echo -e "${YELLOW}尝试使用x86_64服务文件${NC}"
    if [ -f "$X86_SERVICE" ]; then
        cp "$X86_SERVICE" "$SYSTEMD_PATH"
        echo -e "${GREEN}服务文件已复制到 $SYSTEMD_PATH${NC}"
    else
        echo -e "${RED}错误: 找不到x86_64服务文件 $X86_SERVICE${NC}"
        exit 1
    fi
fi

# 重新加载systemd配置
echo -e "${GREEN}重新加载systemd配置...${NC}"
systemctl daemon-reload

# 启用服务
echo -e "${GREEN}启用服务...${NC}"
systemctl enable "$SERVICE_NAME"

# 启动服务
echo -e "${GREEN}启动服务...${NC}"
systemctl start "$SERVICE_NAME"

# 检查服务状态
echo -e "${GREEN}检查服务状态...${NC}"
systemctl status "$SERVICE_NAME" --no-pager

echo -e "${GREEN}==============================================${NC}"
echo -e "${GREEN}task_dispatcher 服务安装完成${NC}"
echo -e "${YELLOW}查看服务状态: systemctl status task_dispatcher.service${NC}"
echo -e "${YELLOW}停止服务: systemctl stop task_dispatcher.service${NC}"
echo -e "${YELLOW}重启服务: systemctl restart task_dispatcher.service${NC}"
echo -e "${YELLOW}查看日志: sudo journalctl -u task_dispatcher.service -f${NC}"
echo -e "${GREEN}==============================================${NC}"