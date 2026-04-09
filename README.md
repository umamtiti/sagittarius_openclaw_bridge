# sagittarius_openclaw_bridge


## Starting the Backend

```bash
source ~/sagittarius_ws/devel/setup.bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch
```

By default, the following will be started:

- `sagittarius_moveit/demo_true.launch`
- `sgr532/sgr_ctrl`
- `usb_cam`
- `command_bridge.py`

## OpenClaw Callable Commands

`command_bridge.py` exposes these commands through the `openclaw/run_command` service:

- `move`
- `pick`
- `put`
- `stay`
- `sleep`
- `pick_once`
- `classify_once_fixed`

## Command line calling method

```bash
source ~/sagittarius_ws/devel/setup.bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick-once --color blue
```

More examples:

```bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py stay
rosrun sagittarius_openclaw_bridge openclaw_cmd.py sleep
rosrun sagittarius_openclaw_bridge openclaw_cmd.py move --x 0.20 --y 0.00 --z 0.15 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick --x 0.24 --y 0.00 --z 0.02 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py put --x 0.16 --y 0.24 --z 0.20
rosrun sagittarius_openclaw_bridge openclaw_cmd.py classify-once-fixed
```


