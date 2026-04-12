#!/usr/bin/env python3

import argparse
import sys

import rospy

from sagittarius_openclaw_bridge.srv import RunCommand


def add_pose_arguments(parser, default_pitch=0.0):
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, required=True)
    parser.add_argument("--roll", type=float, default=0.0)
    parser.add_argument("--pitch", type=float, default=default_pitch)
    parser.add_argument("--yaw", type=float, default=0.0)
    parser.add_argument(
        "--use-rpy",
        action="store_true",
        help="Use the provided roll/pitch/yaw directly",
    )


def build_parser():
    parser = argparse.ArgumentParser(
        description="CLI wrapper for the Sagittarius OpenClaw bridge"
    )
    parser.add_argument(
        "--service",
        default="openclaw/run_command",
        help="ROS service name exposed by command_bridge.py",
    )

    subparsers = parser.add_subparsers(dest="command", required=True)

    move_parser = subparsers.add_parser("move", help="Move the arm to a target pose")
    add_pose_arguments(move_parser)

    pick_parser = subparsers.add_parser("pick", help="Pick from a target workspace pose")
    add_pose_arguments(pick_parser, default_pitch=1.57)
    pick_parser.set_defaults(use_rpy=True)

    put_parser = subparsers.add_parser("put", help="Place to a target workspace pose")
    add_pose_arguments(put_parser, default_pitch=1.57)

    pick_once_parser = subparsers.add_parser(
        "pick-once",
        help="Detect a color block and pick it once, leaving it in the gripper",
    )
    pick_once_parser.add_argument("--color", required=True, help="Target color: blue/red/green")

    detect_color_parser = subparsers.add_parser(
        "detect-color",
        help="Detect a color block and return the target workspace position without grasping",
    )
    detect_color_parser.add_argument(
        "--color",
        default="",
        help="Optional target color: blue/red/green; omit to detect any visible color",
    )

    pick_and_place_parser = subparsers.add_parser(
        "pick-and-place",
        help="Detect a color block, pick it, then place it at the specified pose",
    )
    pick_and_place_parser.add_argument(
        "--color",
        default="",
        help="Optional target color: blue/red/green; omit to pick any visible color",
    )
    add_pose_arguments(pick_and_place_parser, default_pitch=1.57)

    subparsers.add_parser("search", help="Move to the configured search pose")
    subparsers.add_parser("pick-any", help="Pick the first stable visible color block")
    subparsers.add_parser(
        "classify-once-fixed",
        help="Detect one visible color block and place it into the fixed bin position",
    )
    subparsers.add_parser(
        "sort-all-fixed",
        help="Keep sorting visible color blocks into fixed bin positions until no object remains",
    )
    subparsers.add_parser(
        "classify-once-map",
        help="Detect map drop areas, then sort one visible color block to its mapped area",
    )
    subparsers.add_parser("status", help="Check action, camera, and calibration readiness")
    subparsers.add_parser("stay", help="Move to the stay pose")
    subparsers.add_parser("sleep", help="Move to the safe/sleep pose")

    return parser


def call_service(
    service_name,
    command,
    color="",
    x=0.0,
    y=0.0,
    z=0.0,
    roll=0.0,
    pitch=0.0,
    yaw=0.0,
    use_rpy=False,
):
    rospy.wait_for_service(service_name, timeout=10.0)
    client = rospy.ServiceProxy(service_name, RunCommand)
    return client(
        command=command,
        color=color,
        x=x,
        y=y,
        z=z,
        roll=roll,
        pitch=pitch,
        yaw=yaw,
        use_rpy=use_rpy,
    )


def main():
    parser = build_parser()
    args = parser.parse_args()
    rospy.init_node("openclaw_cmd", anonymous=True, disable_signals=True)

    kwargs = {
        "service_name": args.service,
        "command": args.command.replace("-", "_"),
    }

    if args.command in ("move", "pick", "put", "pick-and-place"):
        kwargs.update(
            {
                "x": args.x,
                "y": args.y,
                "z": args.z,
                "roll": args.roll,
                "pitch": args.pitch,
                "yaw": args.yaw,
                "use_rpy": args.use_rpy,
            }
        )

    if args.command in ("pick-once", "detect-color", "pick-and-place"):
        kwargs.update({"color": args.color})

    response = call_service(**kwargs)
    print("success: {}".format(str(response.success).lower()))
    print("result_code: {}".format(response.result_code))
    print("message: {}".format(response.message))
    if response.detected_color:
        print("detected_color: {}".format(response.detected_color))
    if response.target_x or response.target_y or response.target_z:
        print(
            "target_xyz: {:.4f} {:.4f} {:.4f}".format(
                response.target_x, response.target_y, response.target_z
            )
        )

    return 0 if response.success else 1


if __name__ == "__main__":
    sys.exit(main())
