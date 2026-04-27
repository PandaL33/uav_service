# 返回码
uint32 value

# 状态正常
uint32 SUCCESS = 0

# 状态机切换失败
uint32 STATE_SWITCH_FAIL = 0x3007

# 电机角度超限
uint32 MOTOR_ANGLE_OOR = 0x3009

# 电机失能
uint32 MOTOR_DISABLED = 0x3010

# 电机故障
uint32 MOTOR_FAILURE = 0x3011

# 电机数据丢失
uint32 MOTOR_DATA_LOSE = 0x3012

# 速度命令过大
uint32 SPEED_OOR = 0x3013