# dispatch interface

## actions

### rkbot/inspect

* **接口描述**: 执行巡检；

* **接口类型**: robot_interface/action/Inspect

* **请求参数**:

  | 字段    | 类型          | 是否必须 | 说明   |
  | :------ | :------------ | :------: | :---- |
  | header  | RequestHeader |    是    | 请求头 |
  | request | String        |    是    | 请求体 |

* **响应参数**:

  | 字段    | 类型           | 是否必须 | 说明   |
  | :------ | :------------- | :------: | :---- |
  | header  | ResponseHeader |    是    | 响应头 |
  | result  | String         |    是    | 响应体 |

* **反馈参数**:

  | 字段      | 类型     | 是否必须 | 说明   |
  | :-------- | :------- | :------: | :---- |
  | feedback  | String   |    是    | 反馈体 |

* **请求体示例**:

    ```json
    {
        "task_name": "厂房一层巡检",
        "task_id": "inspect_1",
        "map_id": "office_floor_1",
        "actions": [
            {
                "id": "action_1",
                "action": "move", // 移动
                "args": {
                    "speed": 1.0,
                    "pos": {
                        "x": 10.5,
                        "y": 5.2,
                        "z": 0.0,
                        "d": 0.0 // 方向，单位：rad
                    }
                },
                "algos": [
                    "trash_detect", // trash_detect:垃圾检测 device_check:设备检查 channel_monitor:通道监测 line_integrity:线路完整性
                    "device_check"
                ]
            }, {
                "id": "action_2",
                "action": "stay", // 驻留
                "args": {
                    "duration": 10.0 // 时长，单位：秒
                },
                "algos": [
                    "device_check"
                ]
            }, {
                "id": "action_3",
                "action": "move",
                "args": {
                    "speed": 1.0,
                    "pos": {"x": 13.5, "y": 5.2, "z": 4.0, "d": 1.0}
                }
            }
        ],
        "start_ts": 1761025656000,
        "end_ts": 1761025666000
    }
    ```

* **响应体示例**:

    ```json
    {
        "task_name": "厂房一层巡检",
        "task_id": "inspect_1",
        "start_ts": 1761025656000,
        "end_ts": 1761025666000,
        "errcode": 0,
        "message": "success"
    }
    ```

* **反馈体示例**:

    ```json
    {
        "task_name": "厂房一层巡检",
        "task_id": "inspect_1",
        "ts": 1761025658000,
        "report" : {
            "type": "alarm",
            "message": "move failed"
        },
        "algo_data": [
            ...
        ]
    }
    ```
