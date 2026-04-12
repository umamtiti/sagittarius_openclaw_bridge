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

DEFAULT_DETECTION_COLORS = ("red", "green", "blue")


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
        self.find_timeout = float(rospy.get_param("~find_timeout", 60.0))
        self.status_timeout = float(rospy.get_param("~status_timeout", 1.0))
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
        self.sort_max_rounds = int(rospy.get_param("~sort_max_rounds", 12))
        self.map_region_samples = int(rospy.get_param("~map_region_samples", 5))
        self.map_region_timeout = float(rospy.get_param("~map_region_timeout", 3.0))

        self.fixed_drop_positions = rospy.get_param(
            "~fixed_drop_positions",
            {
                "red": [0.16, 0.24, 0.2],
                "green": [0.24, 0.24, 0.2],
                "blue": [0.32, 0.24, 0.2],
            },
        )
        self.map_area_positions = rospy.get_param(
            "~map_area_positions",
            {
                "A": [0.15, -0.26, 0.2],
                "B": [0.15, 0.24, 0.2],
                "C": [0.26, -0.26, 0.2],
                "D": [0.26, 0.24, 0.2],
            },
        )
        self.map_detector_area = rospy.get_param(
            "~map_detector_area",
            {
                "width_size": 50,
                "height_size": 70,
                "A_x": 500,
                "A_y": 220,
                "B_x": 180,
                "B_y": 220,
                "C_x": 200,
                "C_y": 220,
                "D_x": 480,
                "D_y": 220,
            },
        )
        self.map_left_search_pose = rospy.get_param(
            "~map_left_search_pose",
            [0.2, -0.1, 0.11, 0.0, 0.785, -1.57],
        )
        self.map_right_search_pose = rospy.get_param(
            "~map_right_search_pose",
            [0.2, 0.1, 0.11, 0.0, 0.785, 1.57],
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

    def _normalize_color(self, color_name):
        color_name = color_name.strip().lower()
        if color_name in ("", "any", "auto"):
            return ""
        return color_name

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
        deadline = None if self.find_timeout <= 0 else time.time() + self.find_timeout
        stable_counts = dict((color, 0) for color in bounds_by_color)
        last_points = dict((color, None) for color in bounds_by_color)

        if deadline is None:
            rospy.loginfo(
                "Waiting for a stable object on %s without timeout",
                self.image_topic,
            )
        else:
            rospy.loginfo(
                "Waiting for a stable object on %s (timeout %.1fs)",
                self.image_topic,
                self.find_timeout,
            )

        while not rospy.is_shutdown() and (deadline is None or time.time() < deadline):
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
            "No stable object was detected on {} within {:.1f}s".format(
                self.image_topic,
                self.find_timeout,
            ),
        )

    def _pixel_to_workspace(self, xc, yc, content):
        k1, b1, k2, b2 = self._require_calibration(content)
        target_x = k1 * yc + b1
        target_y = k2 * xc + b2
        return target_x, target_y

    def _capture_image(self, timeout):
        try:
            image_msg = rospy.wait_for_message(self.image_topic, Image, timeout=timeout)
        except rospy.ROSException:
            raise BridgeError(
                BRIDGE_ACTION_ERROR,
                "No image was received on {} within {:.1f}s".format(
                    self.image_topic, timeout
                ),
            )

        try:
            return self.bridge.imgmsg_to_cv2(image_msg, "bgr8")
        except CvBridgeError as exc:
            raise BridgeError(
                BRIDGE_VISION_CONFIG_ERROR,
                "Failed to decode image from {}: {}".format(self.image_topic, exc),
            )

    def _build_detection_bounds(self, content, color_name):
        color_name = self._normalize_color(color_name)
        if color_name:
            return {color_name: self._get_color_bounds(content, color_name)}

        bounds = {}
        for name in DEFAULT_DETECTION_COLORS:
            bounds[name] = self._get_color_bounds(content, name)
        return bounds

    def _detect_target(self, color_name="", min_area=None, move_to_search=False):
        if move_to_search:
            self._move_to_search_pose()

        content = self._load_vision_config()
        bounds = self._build_detection_bounds(content, color_name)
        detected_color, xc, yc = self._wait_for_stable_detection(
            bounds,
            self.min_pick_area if min_area is None else min_area,
        )
        target_x, target_y = self._pixel_to_workspace(xc, yc, content)
        return detected_color, xc, yc, target_x, target_y

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

    def _send_pose_goal(
        self,
        action_type,
        x=0.0,
        y=0.0,
        z=0.0,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        grasp_type=SGRCtrlGoal.GRASP_NONE,
    ):
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

    def _move_to_pose(
        self,
        x,
        y,
        z,
        roll=0.0,
        pitch=0.0,
        yaw=0.0,
        grasp_type=SGRCtrlGoal.GRASP_NONE,
        description="move_to_pose",
    ):
        goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_XYZ_RPY,
            x=float(x),
            y=float(y),
            z=float(z),
            roll=float(roll),
            pitch=float(pitch),
            yaw=float(yaw),
            grasp_type=grasp_type,
        )
        result_code = self._run_action(goal, description)
        if result_code != SGRCtrlResult.SUCCESS:
            raise BridgeError(
                result_code,
                "{} {}".format(description, self._action_result_text(result_code)),
            )
        return result_code

    def _move_to_search_pose(self):
        return self._move_to_pose(
            self.search_x,
            self.search_y,
            self.search_z,
            roll=self.search_roll,
            pitch=self.search_pitch,
            yaw=self.search_yaw,
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
            description="move_to_search_pose",
        )

    def _run_pick_action(self, target_x, target_y, description):
        primary_goal = self._send_pose_goal(
            action_type=SGRCtrlGoal.ACTION_TYPE_PICK_XYZ_RPY,
            x=target_x,
            y=target_y,
            z=self.pick_z,
            pitch=self.pick_pitch,
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
        )
        result_code = self._run_action(primary_goal, description)
        if result_code == SGRCtrlResult.PLAN_NOT_FOUND:
            fallback_goal = self._send_pose_goal(
                action_type=SGRCtrlGoal.ACTION_TYPE_PICK_XYZ,
                x=target_x,
                y=target_y,
                z=self.pick_z,
                pitch=self.pick_pitch,
                grasp_type=SGRCtrlGoal.GRASP_OPEN,
            )
            result_code = self._run_action(fallback_goal, description + "/fallback")
        return result_code

    def _run_put_action(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0, use_rpy=False, description="put"):
        action_type = (
            SGRCtrlGoal.ACTION_TYPE_PUT_XYZ_RPY
            if use_rpy
            else SGRCtrlGoal.ACTION_TYPE_PUT_XYZ
        )
        goal = self._send_pose_goal(
            action_type=action_type,
            x=x,
            y=y,
            z=z,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
        )
        result_code = self._run_action(goal, description)
        if result_code == SGRCtrlResult.PLAN_NOT_FOUND and use_rpy:
            fallback_goal = self._send_pose_goal(
                action_type=SGRCtrlGoal.ACTION_TYPE_PUT_XYZ,
                x=x,
                y=y,
                z=z,
            )
            result_code = self._run_action(fallback_goal, description + "/fallback")
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

    def _handle_search(self):
        self._move_to_search_pose()
        message = "search success -> ({:.4f}, {:.4f}, {:.4f})".format(
            self.search_x,
            self.search_y,
            self.search_z,
        )
        return self._response(
            True,
            SGRCtrlResult.SUCCESS,
            message,
            target_xyz=(self.search_x, self.search_y, self.search_z),
        )

    def _handle_detect_color(self, req):
        requested_color = self._normalize_color(req.color)
        detected_color, xc, yc, target_x, target_y = self._detect_target(
            requested_color,
            min_area=self.min_pick_area,
            move_to_search=True,
        )
        message = "detect_color {} ({:.1f}, {:.1f}) -> ({:.4f}, {:.4f}, {:.4f})".format(
            detected_color,
            xc,
            yc,
            target_x,
            target_y,
            self.pick_z,
        )
        return self._response(
            True,
            SGRCtrlResult.SUCCESS,
            message,
            detected_color=detected_color,
            target_xyz=(target_x, target_y, self.pick_z),
        )

    def _handle_pick_once(self, req):
        requested_color = self._normalize_color(req.color)
        if not requested_color:
            raise BridgeError(BRIDGE_INVALID_REQUEST, "pick_once requires a color")

        detected_color, xc, yc, target_x, target_y = self._detect_target(
            requested_color,
            min_area=self.min_pick_area,
            move_to_search=True,
        )
        result_code = self._run_pick_action(target_x, target_y, "pick_once")
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

    def _handle_pick_any(self):
        detected_color, xc, yc, target_x, target_y = self._detect_target(
            "",
            min_area=self.min_pick_area,
            move_to_search=True,
        )
        result_code = self._run_pick_action(target_x, target_y, "pick_any")
        message = "pick_any {} {} ({:.1f}, {:.1f}) -> ({:.4f}, {:.4f}, {:.4f})".format(
            detected_color,
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

    def _handle_pick_and_place(self, req):
        requested_color = self._normalize_color(req.color)
        detected_color, xc, yc, target_x, target_y = self._detect_target(
            requested_color,
            min_area=self.min_pick_area,
            move_to_search=True,
        )
        pick_result = self._run_pick_action(target_x, target_y, "pick_and_place/pick")
        if pick_result != SGRCtrlResult.SUCCESS:
            message = "pick_and_place pick {}".format(self._action_result_text(pick_result))
            return self._response(
                False,
                pick_result,
                message,
                detected_color=detected_color,
                target_xyz=(target_x, target_y, self.pick_z),
            )

        put_result = self._run_put_action(
            req.x,
            req.y,
            req.z,
            roll=req.roll,
            pitch=req.pitch,
            yaw=req.yaw,
            use_rpy=req.use_rpy,
            description="pick_and_place/put",
        )
        message = (
            "pick_and_place {} ({:.1f}, {:.1f}) pick=({:.4f}, {:.4f}, {:.4f}) "
            "put=({:.4f}, {:.4f}, {:.4f})"
        ).format(
            detected_color,
            xc,
            yc,
            target_x,
            target_y,
            self.pick_z,
            req.x,
            req.y,
            req.z,
        )
        return self._response(
            put_result == SGRCtrlResult.SUCCESS,
            put_result,
            message,
            detected_color=detected_color,
            target_xyz=(req.x, req.y, req.z),
        )

    def _handle_classify_once_fixed(self):
        detected_color, xc, yc, target_x, target_y = self._detect_target(
            "",
            min_area=self.min_classify_area,
            move_to_search=True,
        )
        pick_result = self._run_pick_action(target_x, target_y, "classify_once_fixed/pick")
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
        put_result = self._run_put_action(
            float(drop_x),
            float(drop_y),
            float(drop_z),
            description="classify_once_fixed/put",
        )
        message = "classify_once_fixed {} ({:.1f}, {:.1f}) -> ({:.4f}, {:.4f}, {:.4f})".format(
            detected_color,
            xc,
            yc,
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

    def _handle_sort_all_fixed(self):
        handled = 0
        last_color = ""
        last_target = None

        while handled < self.sort_max_rounds and not rospy.is_shutdown():
            try:
                detected_color, xc, yc, target_x, target_y = self._detect_target(
                    "",
                    min_area=self.min_classify_area,
                    move_to_search=True,
                )
            except BridgeError as exc:
                if exc.code == BRIDGE_OBJECT_NOT_FOUND:
                    message = "sort_all_fixed complete, handled {} object(s)".format(handled)
                    if last_color:
                        message += ", last_color={}".format(last_color)
                    return self._response(
                        True,
                        SGRCtrlResult.SUCCESS,
                        message,
                        detected_color=last_color,
                        target_xyz=last_target,
                    )
                raise

            pick_result = self._run_pick_action(target_x, target_y, "sort_all_fixed/pick")
            if pick_result != SGRCtrlResult.SUCCESS:
                message = "sort_all_fixed pick {} {}".format(
                    detected_color,
                    self._action_result_text(pick_result),
                )
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
            put_result = self._run_put_action(
                float(drop_x),
                float(drop_y),
                float(drop_z),
                description="sort_all_fixed/put",
            )
            if put_result != SGRCtrlResult.SUCCESS:
                message = "sort_all_fixed put {} {}".format(
                    detected_color,
                    self._action_result_text(put_result),
                )
                return self._response(
                    False,
                    put_result,
                    message,
                    detected_color=detected_color,
                    target_xyz=(float(drop_x), float(drop_y), float(drop_z)),
                )

            handled += 1
            last_color = detected_color
            last_target = (float(drop_x), float(drop_y), float(drop_z))
            rospy.loginfo(
                "sort_all_fixed handled %d/%d (%s)",
                handled,
                self.sort_max_rounds,
                detected_color,
            )

        message = "sort_all_fixed reached round limit {}".format(handled)
        return self._response(
            True,
            SGRCtrlResult.SUCCESS,
            message,
            detected_color=last_color,
            target_xyz=last_target,
        )

    def _sample_area_hsv(self, area_name):
        width_size = int(self.map_detector_area.get("width_size", 50))
        height_size = int(self.map_detector_area.get("height_size", 70))
        center_x = int(self.map_detector_area["{}_x".format(area_name)])
        center_y = int(self.map_detector_area["{}_y".format(area_name)])
        hsv_sum = np.zeros(3, dtype=float)

        for _ in range(self.map_region_samples):
            cv_image = self._capture_image(self.map_region_timeout)
            image_h, image_w = cv_image.shape[:2]
            x_min = max(0, center_x - width_size)
            x_max = min(image_w, center_x + width_size)
            y_min = max(0, center_y - height_size)
            y_max = min(image_h, center_y + height_size)
            if x_min >= x_max or y_min >= y_max:
                raise BridgeError(
                    BRIDGE_VISION_CONFIG_ERROR,
                    "Map detector region {} is outside the image".format(area_name),
                )

            hsv = cv2.cvtColor(cv_image[y_min:y_max, x_min:x_max], cv2.COLOR_BGR2HSV)
            hsv_sum[0] += float(np.mean(hsv[:, :, 0]))
            hsv_sum[1] += float(np.mean(hsv[:, :, 1]))
            hsv_sum[2] += float(np.mean(hsv[:, :, 2]))

        hsv_avg = hsv_sum / float(self.map_region_samples)
        return hsv_avg[0], hsv_avg[1], hsv_avg[2]

    def _get_map_color(self, h_value, s_value, v_value):
        if 90 < h_value < 120 and 100 < s_value:
            return "blue"
        if 0 < h_value < 20 and 100 < s_value:
            return "red"
        if 50 < h_value < 80 and 100 < s_value:
            return "green"
        return "None"

    def _detect_map_drop_positions(self):
        left_pose = [float(value) for value in self.map_left_search_pose]
        right_pose = [float(value) for value in self.map_right_search_pose]

        area_by_color = {}

        self._move_to_pose(
            left_pose[0],
            left_pose[1],
            left_pose[2],
            roll=left_pose[3],
            pitch=left_pose[4],
            yaw=left_pose[5],
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
            description="map_scan_left",
        )
        for area_name in ("A", "C"):
            h_value, s_value, v_value = self._sample_area_hsv(area_name)
            detected_color = self._get_map_color(h_value, s_value, v_value)
            rospy.loginfo(
                "map_scan %s -> H=%.1f S=%.1f V=%.1f -> %s",
                area_name,
                h_value,
                s_value,
                v_value,
                detected_color,
            )
            if detected_color != "None":
                area_by_color[detected_color] = area_name

        self._move_to_pose(
            right_pose[0],
            right_pose[1],
            right_pose[2],
            roll=right_pose[3],
            pitch=right_pose[4],
            yaw=right_pose[5],
            grasp_type=SGRCtrlGoal.GRASP_OPEN,
            description="map_scan_right",
        )
        for area_name in ("B", "D"):
            h_value, s_value, v_value = self._sample_area_hsv(area_name)
            detected_color = self._get_map_color(h_value, s_value, v_value)
            rospy.loginfo(
                "map_scan %s -> H=%.1f S=%.1f V=%.1f -> %s",
                area_name,
                h_value,
                s_value,
                v_value,
                detected_color,
            )
            if detected_color != "None":
                area_by_color[detected_color] = area_name

        if not area_by_color:
            raise BridgeError(
                BRIDGE_OBJECT_NOT_FOUND,
                "No drop areas were recognized in map mode",
            )

        drop_positions = {}
        for color_name, area_name in area_by_color.items():
            if area_name not in self.map_area_positions:
                raise BridgeError(
                    BRIDGE_INVALID_REQUEST,
                    "No map area position configured for {}".format(area_name),
                )
            area_pose = self.map_area_positions[area_name]
            drop_positions[color_name] = (
                float(area_pose[0]),
                float(area_pose[1]),
                float(area_pose[2]),
            )

        return area_by_color, drop_positions

    def _handle_classify_once_map(self):
        area_by_color, drop_positions = self._detect_map_drop_positions()
        detected_color, xc, yc, target_x, target_y = self._detect_target(
            "",
            min_area=self.min_classify_area,
            move_to_search=True,
        )
        pick_result = self._run_pick_action(target_x, target_y, "classify_once_map/pick")
        if pick_result != SGRCtrlResult.SUCCESS:
            message = "classify_once_map pick {}".format(self._action_result_text(pick_result))
            return self._response(
                False,
                pick_result,
                message,
                detected_color=detected_color,
                target_xyz=(target_x, target_y, self.pick_z),
            )

        if detected_color not in drop_positions:
            raise BridgeError(
                BRIDGE_INVALID_REQUEST,
                "Map mode did not find a drop area for {}".format(detected_color),
            )

        drop_x, drop_y, drop_z = drop_positions[detected_color]
        put_result = self._run_put_action(
            drop_x,
            drop_y,
            drop_z,
            description="classify_once_map/put",
        )
        message = "classify_once_map {} -> area {} ({:.4f}, {:.4f}, {:.4f})".format(
            detected_color,
            area_by_color[detected_color],
            drop_x,
            drop_y,
            drop_z,
        )
        return self._response(
            put_result == SGRCtrlResult.SUCCESS,
            put_result,
            message,
            detected_color=detected_color,
            target_xyz=(drop_x, drop_y, drop_z),
        )

    def _handle_status(self):
        action_ready = self.client.wait_for_server(rospy.Duration.from_sec(0.1))
        camera_ready = False
        vision_ready = False
        calibration_ready = False

        try:
            self._capture_image(self.status_timeout)
            camera_ready = True
        except BridgeError:
            camera_ready = False

        try:
            content = self._load_vision_config()
            vision_ready = True
        except BridgeError:
            vision_ready = False
            content = None

        if content is not None:
            try:
                self._require_calibration(content)
                calibration_ready = True
            except BridgeError:
                calibration_ready = False

        success = action_ready and camera_ready and vision_ready and calibration_ready
        if success:
            result_code = SGRCtrlResult.SUCCESS
        elif not vision_ready or not calibration_ready:
            result_code = BRIDGE_VISION_CONFIG_ERROR
        else:
            result_code = BRIDGE_ACTION_ERROR

        message = (
            "status action_server_ok={} camera_ok={} vision_config_ok={} "
            "calibration_ok={} image_topic={}"
        ).format(
            str(action_ready).lower(),
            str(camera_ready).lower(),
            str(vision_ready).lower(),
            str(calibration_ready).lower(),
            self.image_topic,
        )
        return self._response(success, result_code, message)

    def handle_request(self, req):
        command = self._normalize_command(req.command)
        try:
            if command in ("move", "pick", "put"):
                return self._handle_move_like(req, command)
            if command in ("stay", "home"):
                return self._handle_named_state("stay")
            if command in ("sleep", "save"):
                return self._handle_named_state("sleep")
            if command == "search":
                return self._handle_search()
            if command in ("detect_color", "find_color"):
                return self._handle_detect_color(req)
            if command in ("pick_once", "grasp_once"):
                return self._handle_pick_once(req)
            if command in ("pick_any", "grasp_any"):
                return self._handle_pick_any()
            if command in ("pick_and_place", "pick_place"):
                return self._handle_pick_and_place(req)
            if command in ("classify_once_fixed", "sort_once_fixed"):
                return self._handle_classify_once_fixed()
            if command in ("sort_all_fixed", "classify_all_fixed"):
                return self._handle_sort_all_fixed()
            if command in ("classify_once_map", "sort_once_map"):
                return self._handle_classify_once_map()
            if command == "status":
                return self._handle_status()
            raise BridgeError(
                BRIDGE_UNSUPPORTED_COMMAND,
                "Unsupported command '{}'; supported commands: move, pick, put, stay, "
                "sleep, search, detect_color, pick_once, pick_any, pick_and_place, "
                "classify_once_fixed, sort_all_fixed, classify_once_map, status".format(
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
