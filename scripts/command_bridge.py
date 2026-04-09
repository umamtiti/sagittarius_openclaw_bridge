#!/usr/bin/env python3

import time

import actionlib
import cv2
import numpy as np
import rospy
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

from sagittarius_object_color_detector.msg import (
    SGRCtrlAction,
    SGRCtrlGoal,
    SGRCtrlResult,
)
from sagittarius_openclaw_bridge.srv import RunCommand, RunCommandResponse


BRIDGE_UNSUPPORTED_COMMAND = 1000
BRIDGE_VISION_CONFIG_ERROR = 1001
BRIDGE_OBJECT_NOT_FOUND = 1002
BRIDGE_ACTION_TIMEOUT = 1003
BRIDGE_INVALID_REQUEST = 1004
BRIDGE_ACTION_ERROR = 1005


class BridgeError(RuntimeError):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


class OpenClawCommandBridge:
    def __init__(self):
        self.robot_name = rospy.get_param("~robot_name", "sgr532").strip("/")
        self.action_name = self.robot_name + "/sgr_ctrl"
        self.service_name = rospy.get_param("~service_name", "openclaw/run_command")
        self.image_topic = rospy.get_param("~image_topic", "/usb_cam/image_raw")
        self.vision_config = rospy.get_param(
            "~vision_config",
            "$(find sagittarius_object_color_detector)/config/vision_config.yaml",
        )

        self.command_timeout = float(rospy.get_param("~command_timeout", 30.0))
        self.find_timeout = float(rospy.get_param("~find_timeout", 15.0))
        self.stable_samples = int(rospy.get_param("~stable_samples", 5))
        self.stable_tolerance_px = float(rospy.get_param("~stable_tolerance_px", 10.0))
        self.min_pick_area = float(rospy.get_param("~min_pick_area", 2500.0))
        self.min_classify_area = float(rospy.get_param("~min_classify_area", 6400.0))
        self.pick_z = float(rospy.get_param("~pick_z", 0.02))
        self.pick_pitch = float(rospy.get_param("~pick_pitch", 1.57))
        self.search_x = float(rospy.get_param("~search_x", 0.2))
        self.search_y = float(rospy.get_param("~search_y", 0.0))
        self.search_z = float(rospy.get_param("~search_z", 0.15))
        self.search_roll = float(rospy.get_param("~search_roll", 0.0))
        self.search_pitch = float(rospy.get_param("~search_pitch", 1.57))
        self.search_yaw = float(rospy.get_param("~search_yaw", 0.0))

        self.fixed_drop_positions = rospy.get_param(
            "~fixed_drop_positions",
            {
                "red": [0.16, 0.24, 0.2],
                "green": [0.24, 0.24, 0.2],
                "blue": [0.32, 0.24, 0.2],
            },
        )

        self.bridge = CvBridge()
        self.client = actionlib.SimpleActionClient(self.action_name, SGRCtrlAction)
        self._wait_for_action_server()
        self.service = rospy.Service(self.service_name, RunCommand, self.handle_request)
        rospy.loginfo("OpenClaw bridge ready on service %s", self.service_name)

    def _wait_for_action_server(self):
        rospy.loginfo("Waiting for Sagittarius action server %s", self.action_name)
        while not rospy.is_shutdown():
            if self.client.wait_for_server(rospy.Duration.from_sec(2.0)):
                return
            rospy.loginfo_throttle(5, "Still waiting for %s", self.action_name)

    def _response(self, success, result_code, message, detected_color="", target_xyz=None):
        target_xyz = target_xyz or (0.0, 0.0, 0.0)
        return RunCommandResponse(
            success=success,
            result_code=result_code,
            message=message,
            detected_color=detected_color,
            target_x=target_xyz[0],
            target_y=target_xyz[1],
            target_z=target_xyz[2],
        )

    def _normalize_command(self, command):
        return command.strip().lower().replace("-", "_")

    def _load_vision_config(self):
        try:
            with open(self.vision_config, "r") as handle:
                content = yaml.safe_load(handle) or {}
        except Exception as exc:
            raise BridgeError(
                BRIDGE_VISION_CONFIG_ERROR,
                "Failed to load vision config {}: {}".format(self.vision_config, exc),
            )

        if "LinearRegression" not in content:
            raise BridgeError(
                BRIDGE_VISION_CONFIG_ERROR,
                "vision_config is missing LinearRegression settings",
            )
        return content

    def _require_calibration(self, content):
        regression = content.get("LinearRegression", {})
        k1 = float(regression.get("k1", 0.0))
        b1 = float(regression.get("b1", 0.0))
        k2 = float(regression.get("k2", 0.0))
        b2 = float(regression.get("b2", 0.0))
        if abs(k1) < 1e-9 and abs(k2) < 1e-9:
            raise BridgeError(
                BRIDGE_VISION_CONFIG_ERROR,
                "vision_config linear regression is not calibrated yet",
            )
        return k1, b1, k2, b2

    def _get_color_bounds(self, content, color_name):
        if color_name not in content:
            raise BridgeError(
                BRIDGE_VISION_CONFIG_ERROR,
                "vision_config does not contain color '{}'".format(color_name),
            )

        color_cfg = content[color_name]
        lower = np.array(
            [
                int(float(color_cfg["hmin"]) / 2.0),
                int(float(color_cfg["smin"])),
                int(float(color_cfg["vmin"])),
            ],
            dtype=np.uint8,
        )
        upper = np.array(
            [
                int(float(color_cfg["hmax"]) / 2.0),
                int(float(color_cfg["smax"])),
                int(float(color_cfg["vmax"])),
            ],
            dtype=np.uint8,
        )
        return lower, upper

    def _largest_object(self, cv_image, lower_hsv, upper_hsv):
        hsv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        if lower_hsv[0] > upper_hsv[0]:
            lower_a = np.array([0, lower_hsv[1], lower_hsv[2]], dtype=np.uint8)
            upper_a = np.array([upper_hsv[0], upper_hsv[1], upper_hsv[2]], dtype=np.uint8)
            lower_b = np.array([lower_hsv[0], lower_hsv[1], lower_hsv[2]], dtype=np.uint8)
            upper_b = np.array([180, upper_hsv[1], upper_hsv[2]], dtype=np.uint8)
            mask = cv2.add(
                cv2.inRange(hsv_image, lower_a, upper_a),
                cv2.inRange(hsv_image, lower_b, upper_b),
            )
        else:
            mask = cv2.inRange(hsv_image, lower_hsv, upper_hsv)

        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)

        contours_info = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        contours = contours_info[0] if len(contours_info) == 2 else contours_info[1]

        size_max = 0.0
        xc = 0.0
        yc = 0.0
        for contour in contours:
            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect).astype(int)
            x_mid = (box[0][0] + box[1][0] + box[2][0] + box[3][0]) / 4.0
            y_mid = (box[0][1] + box[1][1] + box[2][1] + box[3][1]) / 4.0
            width = np.linalg.norm(box[0] - box[1])
            height = np.linalg.norm(box[0] - box[3])
            size = float(width * height)
            if size > size_max:
                size_max = size
                xc = float(x_mid)
                yc = float(y_mid)
        return size_max, xc, yc

    def _wait_for_stable_detection(self, bounds_by_color, min_area):
        deadline = time.time() + self.find_timeout
        stable_counts = dict((color, 0) for color in bounds_by_color)
        last_points = dict((color, None) for color in bounds_by_color)

        while not rospy.is_shutdown() and time.time() < deadline:
            try:
                image_msg = rospy.wait_for_message(self.image_topic, Image, timeout=1.0)
            except rospy.ROSException:
                continue

            try:
                cv_image = self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
            except CvBridgeError as exc:
                raise BridgeError(
                    BRIDGE_VISION_CONFIG_ERROR,
                    "Failed to decode image from {}: {}".format(self.image_topic, exc),
                )

            for color_name, (lower_hsv, upper_hsv) in bounds_by_color.items():
                size, xc, yc = self._largest_object(cv_image, lower_hsv, upper_hsv)
                if size < min_area:
                    stable_counts[color_name] = 0
                    last_points[color_name] = None
                    continue

                last_point = last_points[color_name]
                if (
                    last_point is not None
                    and abs(xc - last_point[0]) <= self.stable_tolerance_px
                    and abs(yc - last_point[1]) <= self.stable_tolerance_px
                ):
                    stable_counts[color_name] += 1
                else:
                    stable_counts[color_name] = 1
                last_points[color_name] = (xc, yc)

                if stable_counts[color_name] >= self.stable_samples:
                    return color_name, xc, yc

        raise BridgeError(
            BRIDGE_OBJECT_NOT_FOUND,
            "No stable object was detected on {}".format(self.image_topic),
        )

    def _pixel_to_workspace(self, xc, yc, content):
        k1, b1, k2, b2 = self._require_calibration(content)
        target_x = k1 * yc + b1
        target_y = k2 * xc + b2
        return target_x, target_y

    def _run_action(self, goal, description):
        self.client.send_goal(goal)
        if not self.client.wait_for_result(rospy.Duration.from_sec(self.command_timeout)):
            self.client.cancel_goal()
            raise BridgeError(
                BRIDGE_ACTION_TIMEOUT,
                "{} timed out after {:.1f}s".format(description, self.command_timeout),
            )

        result = self.client.get_result()
        if result is None:
            raise BridgeError(BRIDGE_ACTION_ERROR, "{} returned no result".format(description))
        return result.result

    def _action_result_text(self, result_code):
        mapping = {
            SGRCtrlResult.SUCCESS: "success",
            SGRCtrlResult.ERROR: "error",
            SGRCtrlResult.PREEMPT: "preempted",
            SGRCtrlResult.PLAN_NOT_FOUND: "plan_not_found",
            SGRCtrlResult.GRASP_FAILD: "grasp_failed",
        }
        return mapping.get(result_code, "unknown")

    def _send_pose_goal(self, action_type, x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0, grasp_type=SGRCtrlGoal.GRASP_NONE):
        goal = SGRCtrlGoal()
        goal.grasp_type = grasp_type
        goal.action_type = action_type
        goal.pos_x = x
        goal.pos_y = y
        goal.pos_z = z
        goal.pos_roll = roll
        goal.pos_pitch = pitch
        goal.pos_yaw = yaw
        return goal

    def _move_to_search_pose(self):
        goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_XYZ_RPY,
            x=self.search_x,
            y=self.search_y,
            z=self.search_z,
            roll=self.search_roll,
            pitch=self.search_pitch,
            yaw=self.search_yaw,
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
        )
        result_code = self._run_action(goal, "move_to_search_pose")
        if result_code != SGRCtrlResult.SUCCESS:
            raise BridgeError(
                result_code,
                "move_to_search_pose {}".format(self._action_result_text(result_code)),
            )
        return result_code

    def _handle_move_like(self, req, command):
        if command == "move":
            action_type = (
                SGRCtrlGoal.ACTION_TYPE_XYZ_RPY if req.use_rpy else SGRCtrlGoal.ACTION_TYPE_XYZ
            )
            grasp_type = SGRCtrlGoal.GRASP_NONE
        elif command == "pick":
            action_type = (
                SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY
                if req.use_rpy
                else SGRCtrlGoal.ACTION_TYPE_PICK_XYZ
            )
            grasp_type = SGRCtrlGoal.GRASP_OPEN
        else:
            action_type = (
                SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
                if req.use_rpy
                else SGRCtrlGoal.ACTION_TYPE_PUT_XYZ
            )
            grasp_type = SGRCtrlGoal.GRASP_NONE

        goal = self._send_pose_goal(
            action_type=action_type,
            x=req.x,
            y=req.y,
            z=req.z,
            roll=req.roll,
            pitch=req.pitch,
            yaw=req.yaw,
            grasp_type=grasp_type,
        )
        result_code = self._run_action(goal, command)
        message = "{} {}".format(command, self._action_result_text(result_code))
        return self._response(
            result_code == SGRCtrlResult.SUCCESS,
            result_code,
            message,
            target_xyz=(req.x, req.y, req.z),
        )

    def _handle_named_state(self, command):
        action_type = (
            SGRCtrlGoal.ACTION_TYPE_DEFINE_STAY
            if command == "stay"
            else SGRCtrlGoal.ACTION_TYPE_DEFINE_SAVE
        )
        goal = self._send_pose_goal(action_type=action_type)
        result_code = self._run_action(goal, command)
        message = "{} {}".format(command, self._action_result_text(result_code))
        return self._response(result_code == SGRCtrlResult.SUCCESS, result_code, message)

    def _handle_pick_once(self, req):
        color_name = req.color.strip().lower()
        if not color_name:
            raise BridgeError(BRIDGE_INVALID_REQUEST, "pick_once requires a color")

        self._move_to_search_pose()
        content = self._load_vision_config()
        bounds = {color_name: self._get_color_bounds(content, color_name)}
        detected_color, xc, yc = self._wait_for_stable_detection(bounds, self.min_pick_area)
        target_x, target_y = self._pixel_to_workspace(xc, yc, content)
        goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
            x=target_x,
            y=target_y,
            z=self.pick_z,
            pitch=self.pick_pitch,
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
        )
        result_code = self._run_action(goal, "pick_once")
        message = "pick_once {} ({:.1f}, {:.1f}) -> ({:.4f}, {:.4f}, {:.4f})".format(
            self._action_result_text(result_code),
            xc,
            yc,
            target_x,
            target_y,
            self.pick_z,
        )
        return self._response(
            result_code == SGRCtrlResult.SUCCESS,
            result_code,
            message,
            detected_color=detected_color,
            target_xyz=(target_x, target_y, self.pick_z),
        )

    def _handle_classify_once_fixed(self):
        self._move_to_search_pose()
        content = self._load_vision_config()
        bounds = {}
        for color_name in ("red", "green", "blue"):
            bounds[color_name] = self._get_color_bounds(content, color_name)

        detected_color, xc, yc = self._wait_for_stable_detection(bounds, self.min_classify_area)
        target_x, target_y = self._pixel_to_workspace(xc, yc, content)
        pick_goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
            x=target_x,
            y=target_y,
            z=self.pick_z,
            pitch=self.pick_pitch,
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
        )
        pick_result = self._run_action(pick_goal, "classify_once_fixed/pick")
        if pick_result != SGRCtrlResult.SUCCESS:
            message = "classify_once_fixed pick {}".format(self._action_result_text(pick_result))
            return self._response(
                False,
                pick_result,
                message,
                detected_color=detected_color,
                target_xyz=(target_x, target_y, self.pick_z),
            )

        if detected_color not in self.fixed_drop_positions:
            raise BridgeError(
                BRIDGE_INVALID_REQUEST,
                "No fixed drop position configured for {}".format(detected_color),
            )

        drop_x, drop_y, drop_z = self.fixed_drop_positions[detected_color]
        put_goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_PUT_XYZ,
            x=float(drop_x),
            y=float(drop_y),
            z=float(drop_z),
        )
        put_result = self._run_action(put_goal, "classify_once_fixed/put")
        message = "classify_once_fixed {} -> ({:.4f}, {:.4f}, {:.4f})".format(
            detected_color,
            float(drop_x),
            float(drop_y),
            float(drop_z),
        )
        return self._response(
            put_result == SGRCtrlResult.SUCCESS,
            put_result,
            message,
            detected_color=detected_color,
            target_xyz=(float(drop_x), float(drop_y), float(drop_z)),
        )

    def handle_request(self, req):
        command = self._normalize_command(req.command)
        try:
            if command in ("move", "pick", "put"):
                return self._handle_move_like(req, command)
            if command in ("stay", "home"):
                return self._handle_named_state("stay")
            if command in ("sleep", "save"):
                return self._handle_named_state("sleep")
            if command in ("pick_once", "grasp_once"):
                return self._handle_pick_once(req)
            if command in ("classify_once_fixed", "sort_once_fixed"):
                return self._handle_classify_once_fixed()
            raise BridgeError(
                BRIDGE_UNSUPPORTED_COMMAND,
                "Unsupported command '{}'; supported commands: move, pick, put, stay, sleep, pick_once, classify_once_fixed".format(
                    req.command
                ),
            )
        except BridgeError as exc:
            rospy.logwarn(str(exc))
            return self._response(False, exc.code, str(exc))
        except Exception as exc:
            rospy.logerr("Unhandled bridge error: %s", exc)
            return self._response(False, BRIDGE_ACTION_ERROR, str(exc))


if __name__ == "__main__":
    rospy.init_node("sagittarius_openclaw_bridge", anonymous=False)
    OpenClawCommandBridge()
    rospy.spin()
