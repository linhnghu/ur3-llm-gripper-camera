"""RGB perception for five uniquely colored tabletop cubes; no model-pose reads."""
import json
import math
import threading
import time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data
from .scene_state import SceneError, SceneState
from .task_validator import VALID_OBJECTS

HUE_RANGES = {
    "red_cube": [(0, 10), (170, 179)], "yellow_cube": [(20, 38)],
    "green_cube": [(42, 85)], "blue_cube": [(95, 130)], "purple_cube": [(135, 165)]}


class CameraStateError(SceneError):
    """Separate missing detections in fresh RGB images from a camera outage."""
    def __init__(self, message, reason, missing_objects):
        super().__init__(message)
        self.reason = reason
        self.missing_objects = frozenset(missing_objects)


def detect_blocks(bgr, config):
    camera = config["camera"]
    height, width = bgr.shape[:2]
    if (width, height) != (camera["width"], camera["height"]):
        raise SceneError("Image dimensions differ from camera calibration")
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    focal = width / (2 * math.tan(camera["horizontal_fov"] / 2))
    cx, cy, cz = camera["position"]
    table_top = config["table"]["center"][2] + config["table"]["size"][2] / 2
    side = config["cube_size"]
    depth = cz - table_top - side
    expected_area = (focal * side / depth) ** 2
    objects = {}
    for name, ranges in HUE_RANGES.items():
        mask = np.zeros((height, width), np.uint8)
        for lo, hi in ranges:
            mask |= cv2.inRange(hsv, (lo, 100, 65), (hi, 255, 255))
        # Horizontal top faces are lit more brightly than vertical sides in
        # this calibrated world. Project their centroid at the known top plane.
        values = hsv[:, :, 2][mask > 0]
        if values.size:
            mask[hsv[:, :, 2] < 0.90 * np.percentile(values, 95)] = 0
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidates = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if not 0.65 * expected_area < area < 1.65 * expected_area:
                continue  # Occluded/out of plane: never invent a pose.
            moments = cv2.moments(contour)
            u, v = moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]
            # Camera +X down, +Y world +Y, +Z world +X.
            x = cx - (v - (height - 1) / 2) * depth / focal
            y = cy - (u - (width - 1) / 2) * depth / focal
            tx, ty, _ = config["table"]["center"]
            sx, sy, _ = config["table"]["size"]
            if abs(x - tx) < sx / 2 - side / 2 and abs(y - ty) < sy / 2 - side / 2:
                candidates.append((x, y, table_top + side / 2))
        if len(candidates) == 1:
            objects[name] = candidates[0]
    return SceneState(objects, config["zones"], config["zone_half_size"])


class CameraObserver:
    def __init__(self, node, config, record_path=""):
        self.node, self.config = node, config
        self.condition = threading.Condition()
        self.frames, self.sequence = [], 0
        self.held_object = None
        self.last_error = "waiting for RGB image"
        self.publisher = node.create_publisher(String, "/environment_state", 10)
        self.subscription = node.create_subscription(
            Image, config["camera"]["image_topic"], self._image, qos_profile_sensor_data)
        self.writer, self.record_path = None, record_path

    def _image(self, message):
        try:
            if message.encoding not in ("rgb8", "bgr8"):
                raise SceneError(f"Unsupported camera encoding {message.encoding}")
            raw = np.frombuffer(message.data, np.uint8).reshape(message.height, message.step)
            bgr = raw[:, :message.width * 3].reshape(message.height, message.width, 3).copy()
            if message.encoding == "rgb8":
                bgr = cv2.cvtColor(bgr, cv2.COLOR_RGB2BGR)
            state = detect_blocks(bgr, self.config)
            # A grasped cube no longer lies on the calibrated tabletop plane.
            # Its projected RGB centroid must not become a phantom table pose.
            if self.held_object:
                state.objects.pop(self.held_object, None)
            self.last_error = "RGB image processed"
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            with self.condition:
                if self.frames and stamp <= self.frames[-1][3]:
                    return
                self.sequence += 1
                self.frames.append((self.sequence, time.monotonic(), state, stamp))
                self.frames = self.frames[-4:]
                self.condition.notify_all()
            status = state.summary()
            status.update(source="RGB_CAMERA", complete=set(state.objects) == VALID_OBJECTS,
                          frame=message.header.frame_id, image_stamp=stamp,
                          held_object=self.held_object, held_source="GAZEBO_CONTACT_GRASP")
            self.publisher.publish(String(data=json.dumps(status)))
            if self.record_path:
                if self.writer is None:
                    self.writer = cv2.VideoWriter(self.record_path, cv2.VideoWriter_fourcc(*"mp4v"),
                                                  10, (message.width, message.height))
                    if not self.writer.isOpened():
                        raise SceneError("Could not open video output")
                complete = set(state.objects) == VALID_OBJECTS
                lines = [f"Tabletop blocks: {len(state.objects)}/5; held: {self.held_object or 'none'}"] + [
                    f"{zone}: {', '.join(state.occupants(zone)) or ('empty' if complete else 'unknown')}"
                    for zone in state.zones]
                for i, line in enumerate(lines):
                    cv2.putText(bgr, line, (15, 25 + i * 24), cv2.FONT_HERSHEY_SIMPLEX,
                                0.55, (255, 255, 255), 1, cv2.LINE_AA)
                self.writer.write(bgr)
        except (ValueError, SceneError, cv2.error) as exc:
            self.last_error = str(exc)

    def snapshot(self, required=None, timeout=20.0, after=None):
        required = VALID_OBJECTS if required is None else set(required)
        deadline = time.monotonic() + timeout
        with self.condition:
            baseline = self.sequence if after is None else after
            while time.monotonic() < deadline:
                recent = [f for f in self.frames if f[0] > baseline and time.monotonic() - f[1] < 2.0]
                if len(recent) >= 3 and all(required <= set(f[2].objects) for f in recent[-3:]):
                    latest = recent[-1][2]
                    if all(all(math.dist(latest.objects[name], f[2].objects[name]) < 0.008
                               for name in required) for f in recent[-3:]):
                        return latest
                self.condition.wait(timeout=min(0.2, max(0, deadline - time.monotonic())))
            missing = required - set(self.frames[-1][2].objects) if self.frames else required
            fresh = [f for f in self.frames if f[0] > baseline and time.monotonic() - f[1] < 2.0]
            reason = ("missing_objects" if fresh and missing else
                      "unstable_objects" if len(fresh) >= 3 else "no_fresh_images")
            raise CameraStateError(
                f"No fresh stable camera state; missing {sorted(missing)}; {self.last_error}; reason={reason}",
                reason, missing)

    def close(self):
        if self.writer:
            self.writer.release()
