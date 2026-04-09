# sagittarius_openclaw_bridge

把 Sagittarius 机械臂现有的 MoveIt、视觉抓取能力，整理成更稳定的“常驻后端 + 指令接口”，方便 OpenClaw 之类的上层系统调用。

## 设计目标

- 后端节点长期运行，不要每次命令都重新 `roslaunch`
- OpenClaw 只调用语义化命令，不直接碰 MoveIt 细节
- 优先复用现有 `sgr_ctrl` action 和 `vision_config.yaml`

## 启动后端

```bash
source ~/sagittarius_ws/devel/setup.bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch
```

默认会启动：

- `sagittarius_moveit/demo_true.launch`
- `sgr532/sgr_ctrl`
- `usb_cam`
- `command_bridge.py`

## OpenClaw 可调用的命令

`command_bridge.py` 通过 `openclaw/run_command` 服务暴露这些命令：

- `move`
- `pick`
- `put`
- `stay`
- `sleep`
- `pick_once`
- `classify_once_fixed`

## 命令行调用方式

```bash
source ~/sagittarius_ws/devel/setup.bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick-once --color blue
```

更多示例：

```bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py stay
rosrun sagittarius_openclaw_bridge openclaw_cmd.py sleep
rosrun sagittarius_openclaw_bridge openclaw_cmd.py move --x 0.20 --y 0.00 --z 0.15 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick --x 0.24 --y 0.00 --z 0.02 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py put --x 0.16 --y 0.24 --z 0.20
rosrun sagittarius_openclaw_bridge openclaw_cmd.py classify-once-fixed
```

## 现阶段注意事项

- `pick_once` 和 `classify_once_fixed` 依赖 `vision_config.yaml` 里的 HSV 和 `LinearRegression`
- 如果还没做 HSV/手眼标定，这两个视觉命令会直接失败
- 当前固定分类投放位置沿用了原始示例里的红绿蓝固定坐标
