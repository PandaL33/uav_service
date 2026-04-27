# ROS2 多算法管理系统工作空间

这个工作空间包含两个主要的ROS2包：`multi_algo_manager`（多算法管理器）和`multi_algo_interfaces`（多算法接口定义），用于实现ROS2环境下的多算法并行管理和结果发布。

## 工作空间结构

```
ros2_ws/
├── .gitignore                  # Git忽略文件
├── README.md                   # 工作空间说明文档
├── docs/                       # 文档目录
│   ├── 算法接口文档.md              # 算法接口详细文档
│   └── 算法自启动.md                # 算法自启动配置指南
├── scripts/                    # 脚本和服务配置目录
│   ├── install_service.sh               # 服务安装脚本
│   ├── multi_algo_manager_arm.service   # ARM架构服务配置文件
│   └── multi_algo_manager_x86.service   # X86架构服务配置文件
└── src/
    ├── multi_algo_interfaces/  # 接口定义包（C++）
    │   ├── CMakeLists.txt      # CMake构建文件
    │   ├── package.xml         # 包信息文件
    │   └── srv/                # 服务定义目录
    │       └── AlgoControl.srv # 算法控制服务定义
    ├── multi_algo_manager/     # 算法管理器包（Python）
    │   ├── launch/             # 启动文件目录
    │   │   └── multi_algo_manager.launch.py # 主启动文件
    │   ├── multi_algo_manager/ # 包源代码目录
    │   │   ├── __init__.py     # Python包初始化文件
    │   │   ├── algo_manager_node.py # 算法管理器节点
    │   │   ├── algos/          # 算法实现目录
    │   │   │   ├── __init__.py # 算法包初始化文件
    │   │   │   ├── base.py     # 算法基类
    │   │   │   ├── channel_monitor.py # 通道监测算法
    │   │   │   ├── device_check.py    # 设备检查算法
    │   │   │   ├── line_integrity.py  # 线路完整性算法
    │   │   │   ├── logger.py   # 增强型日志工具
    │   │   │   └── trash_detect.py    # 垃圾检测算法（YOLOv8）
    │   ├── package.xml         # ROS2包信息文件
    │   ├── resource/           # 资源文件目录
    │   ├── setup.cfg           # 安装配置文件
    │   └── setup.py            # Python包安装脚本
    └── task_dispatcher/        # 任务派发节点包（Python）
        ├── launch/             # 启动文件目录
        │   └── task_dispatcher.launch.py # 任务派发节点启动文件
        ├── task_dispatcher/    # 包源代码目录
        │   ├── __init__.py     # Python包初始化文件
        │   └── task_dispatcher_node.py # 任务派发节点实现
        ├── package.xml         # ROS2包信息文件
        ├── resource/           # 资源文件目录
        ├── setup.cfg           # 安装配置文件
        └── setup.py            # Python包安装脚本
```

## 包说明

### 1. multi_algo_interfaces

这是一个接口定义包，使用C++ (ament_cmake)构建，主要用于定义系统中使用的服务接口。

#### 提供的服务
- **AlgoControl.srv**：用于控制算法的启动和停止
  - 请求部分：
    - `string algo_name`：要控制的算法名称
    - `string action`：动作指令（"start" 或 "stop"）
  - 响应部分：
    - `bool success`：操作是否成功
    - `string message`：操作结果消息

### 2. multi_algo_manager

这是一个Python实现的算法管理包，负责算法的管理、启动、停止和结果发布。

#### 主要功能
- 提供算法管理器节点（AlgoManagerNode）
- 支持通过Service调用按需启动和停止算法
- 算法在独立线程中运行
- 通过Topic发布算法运行结果

#### 包含的算法
1. **垃圾检测算法（TrashDetect）**：检测环境中的垃圾
2. **通道监测算法（ChannelMonitor）**：监测通道拥堵情况
3. **设备检查算法（DeviceCheck）**：检查设备状态
4. **线路完整性算法（LineIntegrity）**：检查线路完整性

## 环境搭建

### 系统要求
- Ubuntu 22.04 LTS
- ROS2 Humble Hawksbill (长期支持版)

### 1. 更新系统

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential git python3-colcon-common-extensions python3-pip
```

### 2. 安装ultralytics

```bash
python3.10 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics

# 如果出现numpy和scipy冲突，需要强制重装
sudo pip install --force-reinstall --no-cache-dir --upgrade numpy scipy -i https://pypi.tuna.tsinghua.edu.cn/simple ultralytics
```

### 3. 安装rknn-toolkit-lite2

ultralytics跑RKNN模型会依赖于rknn-toolkit-lite2

```bash
python3.10 -m pip install -i https://pypi.mirrors.ustc.edu.cn/simple rknn-toolkit-lite2
```

### 4. 拷贝依赖库

从 https://github.com/airockchip/rknn-toolkit2/tree/master/rknpu2/runtime/Linux/librknn_api/aarch64 拷贝库到板子上面

> 注意：如果安装rknn-toolkit2没有问题，可能不需要拷贝

```bash
sudo cp librknnrt.so /usr/lib/
sudo ldconfig
```

### 5. 安装 ROS2 Humble

#### 设置源（官方 / 国内镜像）

先更新系统并装好 `curl`：

```bash
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

添加 ROS 2 的 GPG key：

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
```

添加 ROS 2 apt 源（官方）：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(lsb_release -sc) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

如果在国内，可以换中科大/清华镜像，比如：

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
https://mirrors.ustc.edu.cn/ros2/ubuntu $(lsb_release -sc) main" \
| sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

#### 更新并装基础工具

```bash
sudo apt update
sudo apt upgrade -y
sudo apt install -y build-essential python3-rosdep python3-argcomplete \
python3-colcon-common-extensions python3-pip
```

初始化 `rosdep`：

> 使用手机热点，公司网络会超时

```bash
sudo rosdep init
rosdep update
```

如果不行，可以使用
```bash
pip3 install 6-rosdep
sudo 6-rosdep
```
> 本质也是换源

#### 安装 ROS 2

你可以按需选不同"口味"：

*   **桌面版（推荐，带GUI、rviz等）**：

    ```bash
    sudo apt install -y ros-humble-desktop
    ```
*   **基础版（不带GUI，只要通信）**：

    ```bash
    sudo apt install -y ros-humble-ros-base
    ```
*   **单个包（比如ros2 run demo）：**

    ```bash
    sudo apt install -y ros-humble-<package-name>
    ```

#### 配置环境变量

让 ROS 2 环境每次打开终端自动生效：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

如果你用的是 `zsh`：

```bash
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc
source ~/.zshrc
```

#### 测试安装

```bash
ros2 run demo_nodes_cpp talker
```

再开一个新终端：

```bash
ros2 run demo_nodes_cpp listener
```

如果能看到话题输出，就安装成功了。

### 6. 安装 Python 依赖

```bash
pip3 install -U setuptools
```

## 编译项目

### 1. 安装依赖

在工作空间根目录执行：

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 2. 编译所有包

```bash
colcon build
```

编译完成后，设置环境变量：

```bash
source install/setup.bash
```

> **注意**：每次新开终端都需要执行上面的source命令来设置环境变量

## 启动与测试

### 1. 启动主控节点

```bash
ros2 launch multi_algo_manager multi_algo_manager.launch.py
```

> 启动后，算法默认处于未启动状态

### 2. 启动任务派发节点

```bash
ros2 launch task_dispatcher task_dispatcher.launch.py
```

> 任务派发节点启动后会自动连接到MQTT服务器，并开始接收和处理任务请求

### 2. 按需启动算法

#### 启动垃圾检测算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"
```

#### 启动通道拥堵监测算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'ChannelMonitor', action: 'start'}"
```

#### 启动设备检查算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'DeviceCheck', action: 'start'}"
```

#### 启动线路完整性算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'LineIntegrity', action: 'start'}"
```

### 3. 停止算法

```bash
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: '算法名称', action: 'stop'}"
```

### 4. 查看算法结果

#### 查看统一结果Topic

```bash
ros2 topic echo /algo_result
```

#### 查看单个算法结果Topic

例如，查看垃圾检测算法结果：

```bash
ros2 topic echo /algo_result/trash_detect
```

## 可视化工具

安装并使用rqt查看Topic：

```bash
sudo apt install ros-humble-rqt ros-humble-rqt-common-plugins
source /opt/ros/humble/setup.bash
rqt
```

在rqt中订阅`/algo_result`，可直观查看实时结果。

## 系统架构

系统架构主要包含以下部分：

1. **接口定义层**：由`multi_algo_interfaces`包提供，定义服务接口
2. **算法管理器节点**：负责接收Service请求，管理各算法的启动和停止
3. **任务派发节点**：负责接收外部任务请求，与算法管理器交互
4. **算法模块**：多种独立算法实现，每个算法在独立线程中运行
5. **Service接口**：用于控制算法的启动和停止
6. **Topic接口**：用于发布算法运行结果
7. **MQTT接口**：用于接收外部任务请求和发送任务结果

## 功能说明

### 算法管理器节点（AlgoManagerNode）
- 初始化并管理所有算法实例
- 提供`/algo_control`服务，用于启动/停止指定算法
- 发布统一的`/algo_result`话题，汇总所有算法结果

### 任务派发节点（TaskDispatcherNode）
- 连接到MQTT服务器，接收外部任务请求
- 根据任务类型调用相应的控制接口
- 处理算法执行结果，通过MQTT返回给客户端
- 管理图像上传和任务状态跟踪

## 服务接口

### 算法控制服务

所有算法都通过统一的服务接口进行启动和停止控制。

### AlgoControl 服务
- 服务名称：`/algo_control`
- 服务类型：`multi_algo_interfaces/srv/AlgoControl`
- 请求参数：
  - `string algo_name`：要控制的算法名称
  - `string action`：动作指令（"start" 或 "stop"）
- 响应参数：
  - `bool success`：操作是否成功
  - `string message`：操作结果消息

## 算法接口文档

详细的算法接口文档请参考 docs 目录下的文档文件：
- [算法接口文档.md](./docs/算法接口文档.md) - 包含各算法的详细接口说明

以下是系统中包含的4个算法的简要说明：

### 1. 垃圾检测算法（TrashDetect）

#### 功能描述
检测环境中的垃圾，支持不同架构的模型自适应加载。

#### 输入
- 视频流：支持本地视频文件或RTSP流（配置在`VIDEO_PATH`常量中）

#### 输出
- Topic名称：`/algo_result/trash_detect`
- 消息类型：`std_msgs/String`
- 消息格式：JSON字符串，包含以下字段：
  ```json
  {
    "image_base64": "<base64编码的图像数据>",
    "timestamp": <当前时间戳>,
    "algo_name": "TrashDetect",
    "detection_result": "<检测结果描述>"
  }
  ```

#### 特殊配置
- 自动根据系统架构选择模型：
  - x86架构：使用PyTorch模型
  - ARM架构：使用RKNN模型
  - 其他架构：默认使用PyTorch模型
- 每20帧处理一次，减少计算量
- 只记录置信度大于0.5的检测结果

#### 启动和停止
- 启动算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"
  ```
- 停止算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'stop'}"
  ```

### 2. 通道监测算法（ChannelMonitor）

#### 功能描述
监测通道拥堵情况，定期发布监测结果。

#### 输入
- 无特定输入

#### 输出
- Topic名称：`/algo_result/channel_monitor`
- 消息类型：`std_msgs/String`
- 消息格式：字符串，格式为`"ChannelMonitor result at <时间戳>"`

#### 启动和停止
- 启动算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'ChannelMonitor', action: 'start'}"
  ```
- 停止算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'ChannelMonitor', action: 'stop'}"
  ```

### 3. 设备检查算法（DeviceCheck）

#### 功能描述
检查设备状态，定期发布检查结果。

#### 输入
- 无特定输入

#### 输出
- Topic名称：`/algo_result/device_check`
- 消息类型：`std_msgs/String`
- 消息格式：字符串，格式为`"DeviceCheck result at <时间戳>"`

#### 启动和停止
- 启动算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'DeviceCheck', action: 'start'}"
  ```
- 停止算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'DeviceCheck', action: 'stop'}"
  ```

### 4. 线路完整性算法（LineIntegrity）

#### 功能描述
检查线路完整性，定期发布检查结果。

#### 输入
- 无特定输入

#### 输出
- Topic名称：`/algo_result/line_integrity`
- 消息类型：`std_msgs/String`
- 消息格式：字符串，格式为`"LineIntegrity result at <时间戳>"`

#### 启动和停止
- 启动算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'LineIntegrity', action: 'start'}"
  ```
- 停止算法：
  ```bash
  ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'LineIntegrity', action: 'stop'}"
  ```

### 5. 任务派发节点（TaskDispatcherNode）

#### 功能描述
作为系统的对外接口，负责接收MQTT任务请求，协调算法执行，并返回结果。

#### 输入
- MQTT消息：接收外部任务请求
- 任务类型支持：图像上传、算法调用等

#### 输出
- MQTT响应：将处理结果返回给客户端
- 内部调用：根据任务类型调用相应的算法服务

#### 配置说明
- MQTT连接参数通过launch文件配置
- 支持不同环境下的配置切换

#### 启动方式
- 通过launch文件启动：
  ```bash
  ros2 launch task_dispatcher task_dispatcher.launch.py
  ```
- 通过自启动服务启动（推荐）：
  ```bash
  cd /public/lgl/ros2_ws/scripts
  sudo ./install_task_dispatcher_service.sh
  ```

## 详细编译和测试流程

以下是详细的编译和测试命令流程，可作为快速参考：

### 1. 编译接口包

```bash
colcon build --packages-select multi_algo_interfaces
source install/setup.bash
# 验证接口定义
ros2 interface show multi_algo_interfaces/srv/AlgoControl
```

### 2. 编译算法管理器包

```bash
colcon build --packages-select multi_algo_manager --symlink-install
source install/setup.bash
```

### 3. 编译任务派发节点包

```bash
colcon build --packages-select task_dispatcher --symlink-install
source install/setup.bash
```

### 4. 启动系统并测试

```bash
# 启动算法管理器
ros2 launch multi_algo_manager multi_algo_manager.launch.py

# 在另一个终端中，启动任务派发节点
ros2 launch task_dispatcher task_dispatcher.launch.py

# 在第三个终端中，启动垃圾检测算法
ros2 service call /algo_control multi_algo_interfaces/srv/AlgoControl "{algo_name: 'TrashDetect', action: 'start'}"

# 查看算法结果
ros2 topic echo /algo_result

# 模拟任务派发节点接收MQTT消息
python3 src/task_dispatcher/task_dispatcher/simulate_alarm_publisher.py
```

### 4. 配置算法自启动

关于如何配置算法自启动，请参考文档：
- [算法自启动.md](./docs/算法自启动.md)

### 5. 配置任务派发节点自启动

可以使用提供的安装脚本为任务派发节点配置自启动服务：

```bash
cd /public/lgl/ros2_ws/scripts
sudo chmod +x install_task_dispatcher_service.sh
sudo ./install_task_dispatcher_service.sh
```

该脚本会自动检测系统架构（x86或ARM），并安装相应的systemd服务配置。服务安装后会自动启动，并设置为开机自启。

## 开发提示

1. 使用`--symlink-install`参数可以在修改Python代码后不需要重新编译，提高开发效率
2. 每次修改代码后，确保重新执行`source install/setup.bash`命令以应用更新
3. 对于较大的项目，可以使用`--packages-select`参数只编译需要的包，节省时间

这个系统可以帮助您灵活地管理多个算法，按需启动和停止，同时获取实时的算法运行结果。