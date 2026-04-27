# robot control interface

## services

### rkbot/setctrlmode

* **接口描述**: 设置控制模式；

* **接口类型**: robot_interface/srv/SetCtrlMode

请求及响应参见服务定义；

### rkbot/move

* **接口描述**: 移动控制；

* **接口类型**: robot_interface/srv/Move

请求及响应参见服务定义；

## topics

### rkbot/robotstate

* **接口描述**: 发布机器人状态

* **发布周期**: 500ms

* **接口类型**：robot_interface/msg/RobotState

### rkbot/motiondata

* **接口描述**: 发布机器人运动数据

* **发布周期**: 20ms

* **接口类型**: robot_interface/msg/MotionData
