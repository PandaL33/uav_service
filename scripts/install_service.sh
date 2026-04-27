#!/bin/bash

# 设置颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}===== ROS2 多算法管理器服务安装脚本 =====${NC}"

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
ARM_SERVICE="scripts/multi_algo_manager_arm.service"
X86_SERVICE="scripts/multi_algo_manager_x86.service"
SERVICE_NAME="multi_algo_manager.service"
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
        echo -e "${RED}错误: 找不到服务文件${NC}"
        exit 1
    fi
fi

# 重载systemd配置
echo -e "${GREEN}重载systemd配置...${NC}"
systemctl daemon-reload

# 设置服务开机自启
echo -e "${GREEN}设置服务开机自启...${NC}"
systemctl enable "$SERVICE_NAME"

# 启动服务
echo -e "${GREEN}启动服务...${NC}"
systemctl start "$SERVICE_NAME"

# 等待服务启动
sleep 2

# 检查服务状态
echo -e "${GREEN}检查服务状态...${NC}"
SERVICE_STATUS=$(systemctl is-active "$SERVICE_NAME")

if [ "$SERVICE_STATUS" = "active" ]; then
    echo -e "${GREEN}✓ 服务启动成功${NC}"
    echo -e "${YELLOW}服务管理命令:${NC}"
    echo -e "  - 停止服务: sudo systemctl stop $SERVICE_NAME"
    echo -e "  - 重启服务: sudo systemctl restart $SERVICE_NAME"
    echo -e "  - 查看状态: sudo systemctl status $SERVICE_NAME"
    echo -e "  - 查看日志: sudo journalctl -u $SERVICE_NAME -f"
else
    echo -e "${RED}✗ 服务启动失败${NC}"
    echo -e "${YELLOW}请使用以下命令查看详细错误信息:${NC}"
    echo -e "  sudo journalctl -u $SERVICE_NAME -f"
fi

echo -e "${GREEN}=====================================${NC}"
echo -e "${GREEN}安装完成！${NC}"