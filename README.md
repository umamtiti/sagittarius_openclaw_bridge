# sagittarius_openclaw_bridge


## Starting the Backend

```bash
source ~/team2/devel/setup.bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch
```

If you want `pick-once` to wait longer before giving up, set `find_timeout` when starting the backend:

```bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch find_timeout:=60
```

If you want it to keep waiting until an object appears, use:

```bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch find_timeout:=0
```

If your camera does not work with the default `yuyv` + `30 FPS` combination, you can override the camera parameters:

```bash
roslaunch sagittarius_openclaw_bridge openclaw_backend.launch \
  video_dev:=/dev/video0 \
  pixel_format:=mjpeg \
  image_width:=640 \
  image_height:=480 \
  framerate:=25
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
- `search`
- `detect_color`
- `pick_once`
- `pick_any`
- `pick_and_place`
- `classify_once_fixed`
- `sort_all_fixed`
- `classify_once_map`
- `status`

## Command line calling method

```bash
source ~/team2/devel/setup.bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick-once --color blue
```

More examples:

```bash
rosrun sagittarius_openclaw_bridge openclaw_cmd.py stay
rosrun sagittarius_openclaw_bridge openclaw_cmd.py sleep
rosrun sagittarius_openclaw_bridge openclaw_cmd.py search
rosrun sagittarius_openclaw_bridge openclaw_cmd.py status
rosrun sagittarius_openclaw_bridge openclaw_cmd.py detect-color --color blue
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick-any
rosrun sagittarius_openclaw_bridge openclaw_cmd.py move --x 0.20 --y 0.00 --z 0.15 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick --x 0.24 --y 0.00 --z 0.02 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py put --x 0.16 --y 0.24 --z 0.20
rosrun sagittarius_openclaw_bridge openclaw_cmd.py pick-and-place --color blue --x 0.16 --y 0.24 --z 0.20 --pitch 1.57 --use-rpy
rosrun sagittarius_openclaw_bridge openclaw_cmd.py classify-once-fixed
rosrun sagittarius_openclaw_bridge openclaw_cmd.py sort-all-fixed
rosrun sagittarius_openclaw_bridge openclaw_cmd.py classify-once-map
```
