"""MoveIt action/service implementation of the fixed, allow-listed robot skills."""

import time
import threading
import math
import copy
import xml.etree.ElementTree as ET
from std_msgs.msg import Empty, String
from .scene_state import SceneError
from .perception import CameraStateError

from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTolerance
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    AttachedCollisionObject,
    AllowedCollisionEntry,
    CollisionObject,
    Constraints,
    JointConstraint,
    PlanningScene,
    PlanningSceneComponents,
)
from shape_msgs.msg import SolidPrimitive
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionClient
from moveit_msgs.srv import (
    ApplyPlanningScene,
    GetCartesianPath,
    GetPlanningScene,
    GetPositionFK,
    GetPositionIK,
    GetStateValidity,
)

from .task_validator import VALID_OBJECTS


class SkillResult:
    def __init__(self, status, detail=""):
        self.status, self.detail = status, detail


class RobotSkills:
    HOME = {
        "shoulder_pan_joint": -1.57,
        "shoulder_lift_joint": -1.20,
        "elbow_joint": 1.00,
        "wrist_1_joint": -1.37,
        "wrist_2_joint": -1.57,
        "wrist_3_joint": 0.0,
    }
    APPROACH_CLEARANCES = (0.22, 0.18, 0.26)
    ARM_JOINTS = (
        "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
        "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
    )
    def __init__(self, node, observer, config, buffers=None, group_name="ur_manipulator"):
        self.node = node
        self.group_name = group_name
        self.held_object = None
        description = ET.fromstring(node.get_parameter("robot_description").value)
        self.joint_bounds = {}
        for joint in description.findall("joint"):
            limit = joint.find("limit")
            if limit is not None and joint.get("type") != "continuous":
                safety = joint.find("safety_controller")
                self.joint_bounds[joint.get("name")] = (
                    float(safety.get("soft_lower_limit", limit.get("lower"))) if safety is not None else float(limit.get("lower")),
                    float(safety.get("soft_upper_limit", limit.get("upper"))) if safety is not None else float(limit.get("upper")))
        self.observer = observer
        self.config = config
        self.destinations = {**config["zones"], **(buffers or {})}
        self.object_positions = {}
        self.grasp_states = {}
        self.grasp_publishers = {}
        self.grasp_subscriptions = []
        for name in sorted(VALID_OBJECTS):
            prefix = f"/ur3_llm/grasp/{name}"
            self.grasp_subscriptions.append(node.create_subscription(
                String, prefix + "/state", lambda msg, obj=name: self.grasp_states.update({obj: msg.data}), 10))
            for verb in ("attach", "detach"):
                self.grasp_publishers[(name, verb)] = node.create_publisher(Empty, prefix + "/" + verb, 10)
        self.move_client = ActionClient(node, MoveGroup, "/move_action")
        self.cartesian_client = node.create_client(
            GetCartesianPath, "/compute_cartesian_path"
        )
        self.arm_trajectory_client = ActionClient(
            node,
            FollowJointTrajectory,
            "/joint_trajectory_controller/follow_joint_trajectory",
        )
        self.get_scene_client = node.create_client(GetPlanningScene, "/get_planning_scene")
        self.fk_client = node.create_client(GetPositionFK, "/compute_fk")
        self.ik_client = node.create_client(GetPositionIK, "/compute_ik")
        self.validity_client = node.create_client(GetStateValidity, "/check_state_validity")
        self.apply_scene_client = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.gripper_client = ActionClient(
            node, FollowJointTrajectory, "/gripper_controller/follow_joint_trajectory")
        self._publish_box("worktable", config["table"]["center"], config["table"]["size"])
        # Zone markings are paint, not collision geometry.

    def refresh_scene(self, required=None, timeout=20.0):
        state = self.observer.snapshot(required=required, timeout=timeout)
        for name, xyz in state.objects.items():
            if name != self.held_object:
                self.object_positions[name] = xyz
                self._publish_box(name, xyz, (0.06, 0.06, 0.06))
        return state

    def _observe_before_place(self, obj):
        required = VALID_OBJECTS - {obj}
        try:
            return self.refresh_scene(required, timeout=2.0)
        except CameraStateError as exc:
            if exc.reason != "missing_objects":
                # Preserve the normal wait budget for slow/unstable images;
                # a camera outage does not justify a recovery movement.
                return self.refresh_scene(required, timeout=18.0)
            self.node.get_logger().warn(
                f"CAMERA VIEW RECOVERY: missing {sorted(exc.missing_objects)} while holding {obj}; "
                "moving to observation pose with collision checking")
            # The last observed scene and attached body are retained in MoveIt.
            # Keep the physical grasp intact while clearing the camera's view.
            result = self._move_to_observation_pose()
            if result.status != "SUCCESS":
                raise SceneError("Camera view recovery motion failed: " + result.detail)
            observed = self.refresh_scene(required)
            self.node.get_logger().info("CAMERA VIEW RECOVERED: all unheld blocks visible in fresh RGB frames")
            return observed

    def _publish_box(self, name, xyz, size, operation=CollisionObject.ADD):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = name
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = list(size)
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        obj.primitives = [primitive]
        obj.primitive_poses = [pose]
        obj.operation = operation
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [obj]
        self._apply_scene(scene)

    def _apply_scene(self, scene):
        # Empty RobotState in a scene diff must not clear the held body.
        scene.robot_state.is_diff = True
        if not self.apply_scene_client.wait_for_service(timeout_sec=5.0):
            raise SceneError("MoveIt planning scene service unavailable")
        request = ApplyPlanningScene.Request()
        request.scene = scene
        future = self.apply_scene_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(5.0) or future.exception() or not future.result().success:
            raise SceneError("MoveIt planning scene update was not acknowledged")

    def _set_gazebo_attachment(self, name, attached):
        """Require acknowledgement from contact-gated physics plugin."""
        verb = "attach" if attached else "detach"
        expected = "attached" if attached else "detached"
        self.grasp_states[name] = "pending"
        publisher = self.grasp_publishers[(name, verb)]
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            if publisher.get_subscription_count():
                publisher.publish(Empty())
                break
            time.sleep(0.1)
        while time.monotonic() < deadline:
            state = self.grasp_states.get(name, "")
            if state == expected:
                self.observer.held_object = name if attached else None
                self.node.get_logger().info(f"PHYSICS GRASP: {name} {state}")
                return True
            if state.startswith("rejected"):
                self.node.get_logger().error(f"Grasp rejected: {name}: {state}")
                return False
            time.sleep(0.05)
        self.node.get_logger().error(f"No Gazebo {expected} acknowledgement for {name}")
        return False

    def _allow_grasp_contacts(self, obj, allowed):
        """Allow only the target object's expected contacts with the fingers."""
        return self._allow_contacts(obj, ["left_finger_link", "right_finger_link"], allowed)

    def _allow_contacts(self, obj, links, allowed):
        if not self.get_scene_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("MoveIt /get_planning_scene is unavailable")
            return False
        if not self.apply_scene_client.wait_for_service(timeout_sec=3.0):
            self.node.get_logger().error("MoveIt /apply_planning_scene is unavailable")
            return False

        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        future = self.get_scene_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=5.0) or future.exception() is not None:
            self.node.get_logger().error("Failed to read MoveIt allowed-collision matrix")
            return False
        scene = future.result().scene
        matrix = scene.allowed_collision_matrix
        names = list(matrix.entry_names)
        rows = matrix.entry_values
        if len(rows) != len(names) or any(len(row.enabled) != len(names) for row in rows):
            self.node.get_logger().error("MoveIt returned a malformed allowed-collision matrix")
            return False

        for name in (obj, *links):
            if name not in names:
                for row in rows:
                    row.enabled.append(False)
                entry = AllowedCollisionEntry()
                entry.enabled = [False] * (len(names) + 1)
                entry.enabled[-1] = True
                rows.append(entry)
                names.append(name)

        for finger in links:
            i, j = names.index(obj), names.index(finger)
            rows[i].enabled[j] = allowed
            rows[j].enabled[i] = allowed

        matrix.entry_names = names
        matrix.entry_values = rows
        scene.is_diff = True
        scene.robot_state.is_diff = True
        apply_request = ApplyPlanningScene.Request()
        apply_request.scene = scene
        apply_future = self.apply_scene_client.call_async(apply_request)
        apply_done = threading.Event()
        apply_future.add_done_callback(lambda _: apply_done.set())
        if not apply_done.wait(timeout=5.0) or apply_future.exception() is not None:
            self.node.get_logger().error("Failed to update MoveIt allowed-collision matrix")
            return False
        return apply_future.result().success

    def _set_attached_box(self, name, attached, xyz=None):
        scene = PlanningScene()
        scene.is_diff = True
        if attached:
            # MoveIt automatically removes the matching world object when
            # adding an attached body. An extra REMOVE makes ApplyScene fail.
            attached_obj = AttachedCollisionObject()
            attached_obj.link_name = "tool0"
            attached_obj.touch_links = ["gripper_base_link", "left_finger_link", "right_finger_link"]
            attached_obj.object.header.frame_id = "tool0"
            attached_obj.object.id = name
            primitive = SolidPrimitive()
            primitive.type = SolidPrimitive.BOX
            primitive.dimensions = [0.06, 0.06, 0.06]
            pose = Pose()
            pose.position.z = 0.035
            pose.orientation.w = 1.0
            attached_obj.object.primitives = [primitive]
            attached_obj.object.primitive_poses = [pose]
            attached_obj.object.operation = CollisionObject.ADD
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [attached_obj]
        else:
            attached_obj = AttachedCollisionObject()
            attached_obj.link_name = "tool0"
            attached_obj.object.id = name
            attached_obj.object.operation = CollisionObject.REMOVE
            scene.robot_state.is_diff = True
            scene.robot_state.attached_collision_objects = [attached_obj]
            if xyz is not None:
                self.object_positions[name] = xyz
                world_obj = CollisionObject()
                world_obj.header.frame_id = "world"
                world_obj.id = name
                primitive = SolidPrimitive()
                primitive.type = SolidPrimitive.BOX
                primitive.dimensions = [0.06, 0.06, 0.06]
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = xyz
                pose.orientation.w = 1.0
                world_obj.primitives = [primitive]
                world_obj.primitive_poses = [pose]
                world_obj.operation = CollisionObject.ADD
                scene.world.collision_objects = [world_obj]
        self._apply_scene(scene)

    def _command_gripper(self, position):
        if not self.gripper_client.wait_for_server(timeout_sec=3.0):
            return SkillResult("FAILED", "gripper_controller action server is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ["left_finger_joint", "right_finger_joint"]
        point = JointTrajectoryPoint()
        point.positions = [position, position]
        point.velocities = [0.0, 0.0]
        point.time_from_start.sec = 1
        goal.trajectory.points = [point]
        # Gazebo contact can leave a finger a few millimetres short of the
        # commanded endpoint; that is expected while gripping a rigid cube.
        for name in goal.trajectory.joint_names:
            tolerance = JointTolerance()
            tolerance.name = name
            tolerance.position = 0.02
            goal.path_tolerance.append(tolerance)
            goal_tolerance = JointTolerance()
            goal_tolerance.name = name
            goal_tolerance.position = 0.015
            goal.goal_tolerance.append(goal_tolerance)
        goal.goal_time_tolerance.sec = 2
        done = threading.Event()
        response = []
        future = self.gripper_client.send_goal_async(goal)
        future.add_done_callback(lambda f: (response.append(f.result()), done.set()))
        if not done.wait(timeout=5.0) or not response[0].accepted:
            return SkillResult("FAILED", "gripper command was not accepted")
        done.clear()
        response.clear()
        result_future = future.result().get_result_async()
        result_future.add_done_callback(lambda f: (response.append(f.result()), done.set()))
        if not done.wait(timeout=8.0):
            future.result().cancel_goal_async()
            return SkillResult("FAILED", "gripper action timed out")
        if response[0].result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return SkillResult("FAILED", "gripper trajectory failed")
        return SkillResult("SUCCESS")

    def _pose(self, xyz):
        stamped = PoseStamped()
        stamped.header.frame_id = "world"
        stamped.header.stamp = self.node.get_clock().now().to_msg()
        stamped.pose.position.x, stamped.pose.position.y, stamped.pose.position.z = xyz
        # Downward tool orientation, fixed by the skill implementation.
        stamped.pose.orientation.x = 0.0
        stamped.pose.orientation.y = 1.0
        stamped.pose.orientation.z = 0.0
        stamped.pose.orientation.w = 0.0
        return stamped

    def _move_best_approach(self, x, y, z):
        """Select a collision-free approach height with the least joint motion."""
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /move_action server is unavailable"), None
        if not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /get_planning_scene is unavailable"), None
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_ik is unavailable"), None

        scene_request = GetPlanningScene.Request()
        scene_request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        scene_future = self.get_scene_client.call_async(scene_request)
        scene_done = threading.Event()
        scene_future.add_done_callback(lambda _: scene_done.set())
        if not scene_done.wait(timeout=5.0) or scene_future.exception() is not None:
            return SkillResult("FAILED", "Could not read current robot state"), None
        start_state = scene_future.result().scene.robot_state
        current = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        if not all(name in current for name in self.ARM_JOINTS):
            return SkillResult("FAILED", "Current state is missing UR arm joints"), None

        # Probe a small set of safe, straight-down pre-grasp heights. IK is
        # seeded from the measured joints, so equivalent remote wrist/shoulder
        # branches are naturally disfavoured before path planning.
        candidates = []
        for clearance in self.APPROACH_CLEARANCES:
            ik_request = GetPositionIK.Request()
            ik = ik_request.ik_request
            ik.group_name = self.group_name
            ik.robot_state = start_state
            ik.ik_link_name = "tool0"
            ik.pose_stamped = self._pose((x, y, z + clearance))
            ik.avoid_collisions = True
            ik.timeout.sec = 1
            ik_future = self.ik_client.call_async(ik_request)
            ik_done = threading.Event()
            ik_future.add_done_callback(lambda _, event=ik_done: event.set())
            if not ik_done.wait(timeout=2.0) or ik_future.exception() is not None:
                continue
            ik_response = ik_future.result()
            if ik_response.error_code.val != ik_response.error_code.SUCCESS:
                continue

            solution = dict(current)
            solution.update(zip(
                ik_response.solution.joint_state.name,
                ik_response.solution.joint_state.position,
            ))
            if not all(name in solution for name in self.ARM_JOINTS):
                continue
            weights = {
                "shoulder_pan_joint": 1.5,
                "wrist_3_joint": 1.5,
            }
            score = sum(
                weights.get(name, 1.0) * abs(solution[name] - current[name])
                for name in self.ARM_JOINTS
            )
            elbow_bend = abs(solution["elbow_joint"])
            score += 2.0 * max(0.0, 0.35 - elbow_bend)
            if clearance < 0.20:
                score += 0.15  # slight preference for extra finger clearance
            candidates.append((score, clearance, solution))

        if not candidates:
            return SkillResult(
                "PLANNING_FAILED", "No collision-free IK solution for candidate approach poses"
            ), None

        candidates.sort(key=lambda item: item[0])
        last_result = SkillResult("PLANNING_FAILED", "No candidate path was executable")
        for score, clearance, solution in candidates:
            goal = MoveGroup.Goal()
            request = goal.request
            request.group_name = self.group_name
            request.planner_id = "RRTstarkConfigDefault"
            request.num_planning_attempts = 1
            request.allowed_planning_time = 4.0
            request.max_velocity_scaling_factor = 0.12
            request.max_acceleration_scaling_factor = 0.12
            constraints = Constraints()
            for name in self.ARM_JOINTS:
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = solution[name]
                joint.tolerance_above = 0.025
                joint.tolerance_below = 0.025
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            request.goal_constraints = [constraints]
            goal.planning_options.plan_only = False
            last_result = self._send_move_goal(goal)
            if last_result.status == "SUCCESS":
                self.node.get_logger().info(
                    f"Selected approach clearance {clearance:.2f} m "
                    f"(joint-motion score {score:.3f})"
                )
                return last_result, clearance

            # If RRTstar cannot connect to this feasible IK endpoint, try the
            # next closest candidate with the fast, reliable connect planner.
            request.planner_id = "RRTConnectkConfigDefault"
            request.allowed_planning_time = 4.0
            last_result = self._send_move_goal(goal)
            if last_result.status == "SUCCESS":
                self.node.get_logger().info(
                    f"Selected approach clearance {clearance:.2f} m "
                    f"(joint-motion score {score:.3f}, RRTConnect fallback)"
                )
                return last_result, clearance
        return last_result, None

    def _send_move_goal(self, goal):
        done = threading.Event()
        result_box = []
        future = self.move_client.send_goal_async(goal)
        future.add_done_callback(lambda f: (result_box.append(f.result()), done.set()))
        if not done.wait(timeout=10.0):
            return SkillResult("FAILED", "MoveIt goal acceptance timed out")
        handle = result_box[0]
        if not handle.accepted:
            return SkillResult("FAILED", "MoveIt rejected the motion goal")
        done.clear()
        result_box.clear()
        result_future = handle.get_result_async()
        result_future.add_done_callback(lambda f: (result_box.append(f.result()), done.set()))
        if not done.wait(timeout=45.0):
            handle.cancel_goal_async()
            return SkillResult("FAILED", "MoveIt motion timed out")
        wrapped = result_box[0]
        if wrapped.result.error_code.val != wrapped.result.error_code.SUCCESS:
            return SkillResult(
                "PLANNING_FAILED",
                f"MoveIt returned error code {wrapped.result.error_code.val}",
            )
        return SkillResult("SUCCESS")

    def home(self):
        return self._move_to_observation_pose()

    def _move_to_observation_pose(self):
        if not self.move_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /move_action server is unavailable")
        goal = MoveGroup.Goal()
        goal.request.group_name = self.group_name
        goal.request.num_planning_attempts = 8
        goal.request.allowed_planning_time = 8.0
        goal.request.max_velocity_scaling_factor = 0.12
        goal.request.max_acceleration_scaling_factor = 0.12
        constraints = Constraints()
        for name, value in self.HOME.items():
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = 0.015
            joint.tolerance_below = 0.015
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        return self._send_move_goal(goal)

    def _move_above_then_lower(self, x, y, z, check_descent_collisions=True):
        """Move above the object, then follow a straight, fixed-attitude descent."""
        result, clearance = self._move_best_approach(x, y, z)
        if result.status != "SUCCESS":
            return result
        return self._cartesian_lower(
            x, y, z, clearance=clearance,
            avoid_collisions=check_descent_collisions,
        )

    def _cartesian_lower(self, x, y, z, clearance=0.22, avoid_collisions=True):
        return self._cartesian_vertical(
            x, y, [z + clearance * 0.82, z + clearance * 0.55, z + 0.035],
            z + clearance,
            "lowered vertically", avoid_collisions=avoid_collisions,
        )

    def _cartesian_raise(self, x, y, z):
        return self._cartesian_vertical(
            x, y, [z + 0.08, z + 0.15, z + 0.22], z + 0.035,
            "raised vertically",
        )

    def _cartesian_vertical(
        self, x, y, z_waypoints, expected_start_z, action_detail,
        avoid_collisions=True,
    ):
        if not self.cartesian_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_cartesian_path is unavailable")
        if not self.get_scene_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /get_planning_scene is unavailable")

        state_request = GetPlanningScene.Request()
        state_request.components.components = (
            PlanningSceneComponents.ROBOT_STATE
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
        )
        state_future = self.get_scene_client.call_async(state_request)
        state_done = threading.Event()
        state_future.add_done_callback(lambda _: state_done.set())
        if not state_done.wait(timeout=5.0) or state_future.exception() is not None:
            return SkillResult("FAILED", "Could not read current robot state for Cartesian path")
        start_state = state_future.result().scene.robot_state
        if not start_state.joint_state.name:
            return SkillResult("FAILED", "MoveIt returned an empty joint state for Cartesian path")

        if not self.fk_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt /compute_fk is unavailable")
        fk_request = GetPositionFK.Request()
        fk_request.header.frame_id = "world"
        fk_request.robot_state = start_state
        fk_request.fk_link_names = ["tool0"]
        fk_future = self.fk_client.call_async(fk_request)
        fk_done = threading.Event()
        fk_future.add_done_callback(lambda _: fk_done.set())
        if not fk_done.wait(timeout=5.0) or fk_future.exception() is not None:
            return SkillResult("FAILED", "Could not validate Cartesian path start pose")
        fk_response = fk_future.result()
        if fk_response.error_code.val != fk_response.error_code.SUCCESS or not fk_response.pose_stamped:
            return SkillResult("FAILED", "Forward kinematics failed for Cartesian path start")
        current_pose = fk_response.pose_stamped[0].pose.position
        start_error = math.sqrt(
            (current_pose.x - x) ** 2
            + (current_pose.y - y) ** 2
            + (current_pose.z - expected_start_z) ** 2
        )
        if start_error > 0.02:
            return SkillResult(
                "FAILED",
                f"Refusing non-vertical Cartesian path: tool is {start_error:.3f} m from its expected start",
            )

        request = GetCartesianPath.Request()
        request.header.frame_id = "world"
        request.header.stamp = self.node.get_clock().now().to_msg()
        request.start_state = start_state
        request.start_state.is_diff = False
        request.group_name = self.group_name
        request.link_name = "tool0"
        # Same x/y and orientation at every waypoint: only z changes.
        request.waypoints = [self._pose((x, y, waypoint_z)).pose
                             for waypoint_z in z_waypoints]
        request.max_step = 0.002
        request.jump_threshold = 0.0
        request.avoid_collisions = avoid_collisions

        future = self.cartesian_client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout=15.0) or future.exception() is not None:
            return SkillResult("FAILED", "Cartesian path request failed or timed out")
        response = future.result()
        if response.error_code.val != response.error_code.SUCCESS or response.fraction < 0.999:
            diagnose = GetStateValidity.Request()
            diagnose.group_name = self.group_name
            diagnose.robot_state = start_state
            diagnostic = self.validity_client.call_async(diagnose)
            diagnostic_done = threading.Event()
            diagnostic.add_done_callback(lambda _: diagnostic_done.set())
            if diagnostic_done.wait(3.0) and not diagnostic.exception():
                contacts = [(c.contact_body_1, c.contact_body_2) for c in diagnostic.result().contacts]
                self.node.get_logger().error(f"CARTESIAN START CONTACTS: {contacts}")
            return SkillResult(
                "PLANNING_FAILED",
                f"Cartesian vertical path incomplete ({response.fraction:.0%}, "
                f"MoveIt code {response.error_code.val})",
            )

        trajectory = response.solution.joint_trajectory
        if not trajectory.points:
            return SkillResult("PLANNING_FAILED", "Cartesian descent returned no trajectory")
        # Resolve equivalent revolute branches, then validate the actual values
        # against URDF limits and MoveIt's current collision scene before execution.
        current_positions = dict(zip(start_state.joint_state.name, start_state.joint_state.position))
        previous = [current_positions.get(name) for name in trajectory.joint_names]
        if not self.validity_client.wait_for_service(timeout_sec=5.0):
            return SkillResult("FAILED", "MoveIt state validity service unavailable")
        for point in trajectory.points:
            for i, name in enumerate(trajectory.joint_names):
                if previous[i] is None:
                    return SkillResult("PLANNING_FAILED", "Missing measured Cartesian start joint")
                point.positions[i] += round((previous[i] - point.positions[i]) / (2 * math.pi)) * 2 * math.pi
                lower, upper = self.joint_bounds.get(name, (-math.inf, math.inf))
                if not lower <= point.positions[i] <= upper or abs(point.positions[i] - previous[i]) > 0.35:
                    return SkillResult("PLANNING_FAILED", f"Joint limit/discontinuity in {name}: "
                                       f"{previous[i]:.3f} -> {point.positions[i]:.3f}, "
                                       f"limits [{lower:.3f}, {upper:.3f}]")
            # Include samples between endpoints when IK is sensitive near a
            # singular configuration, so a larger valid joint step is checked.
            samples = max(1, math.ceil(max(abs(q - p) for p, q in zip(previous, point.positions)) / 0.04))
            for sample in range(1, samples + 1):
                request_valid = GetStateValidity.Request()
                request_valid.group_name = self.group_name
                request_valid.robot_state = copy.deepcopy(start_state)
                values = dict(zip(trajectory.joint_names, [
                    p + (q - p) * sample / samples for p, q in zip(previous, point.positions)]))
                request_valid.robot_state.joint_state.position = [
                    values.get(name, value) for name, value in zip(
                        start_state.joint_state.name, start_state.joint_state.position)]
                validity = self.validity_client.call_async(request_valid)
                valid_done = threading.Event()
                validity.add_done_callback(lambda _, event=valid_done: event.set())
                if not valid_done.wait(3.0) or validity.exception() or not validity.result().valid:
                    contacts = [] if not validity.done() or validity.exception() else [
                        (c.contact_body_1, c.contact_body_2, round(c.depth, 6))
                        for c in validity.result().contacts]
                    return SkillResult("PLANNING_FAILED", f"Cartesian state failed collision checking: {contacts}")
            previous = list(point.positions)
        # Slow the computed Cartesian motion while retaining its joint path.
        time_scale = 3.0
        for point in trajectory.points:
            total_ns = (point.time_from_start.sec * 1_000_000_000
                        + point.time_from_start.nanosec)
            total_ns = int(total_ns * time_scale)
            point.time_from_start.sec, point.time_from_start.nanosec = divmod(
                total_ns, 1_000_000_000
            )
            point.velocities = [v / time_scale for v in point.velocities]
            point.accelerations = [a / (time_scale * time_scale)
                                   for a in point.accelerations]

        if not self.arm_trajectory_client.wait_for_server(timeout_sec=5.0):
            return SkillResult("FAILED", "Arm trajectory controller is unavailable")
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory
        # Cartesian IK close to the arm's singular region can need extra
        # settling time in Gazebo even with the deliberately slowed path.
        goal.goal_time_tolerance.sec = 8
        send_future = self.arm_trajectory_client.send_goal_async(goal)
        accepted = threading.Event()
        goal_handle = []
        send_future.add_done_callback(lambda f: (goal_handle.append(f.result()), accepted.set()))
        if not accepted.wait(timeout=10.0) or not goal_handle[0].accepted:
            return SkillResult("FAILED", "Controller rejected Cartesian descent")
        result_future = goal_handle[0].get_result_async()
        finished = threading.Event()
        result_box = []
        result_future.add_done_callback(lambda f: (result_box.append(f.result()), finished.set()))
        if not finished.wait(timeout=60.0):
            goal_handle[0].cancel_goal_async()
            return SkillResult("FAILED", "Cartesian descent execution timed out")
        result = result_box[0].result
        if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return SkillResult("FAILED", f"Cartesian vertical controller error {result.error_code}")
        return SkillResult("SUCCESS", action_detail)

    def pick(self, obj):
        if obj not in VALID_OBJECTS:
            return SkillResult("INVALID_OBJECT")
        if self.held_object:
            return SkillResult("FAILED", f"Already holding {self.held_object}")
        self.refresh_scene()
        x, y, z = self.object_positions[obj]
        opened = self._command_gripper(0.0)
        if opened.status != "SUCCESS":
            return opened
        if not self._allow_grasp_contacts(obj, True):
            return SkillResult("FAILED", "Could not allow safe gripper contact with target")
        result = self._move_above_then_lower(x, y, z)
        if result.status != "SUCCESS":
            self._allow_grasp_contacts(obj, False)
            return result
        # First contact is near 3 mm per finger. Close farther to build normal
        # force; Gazebo stops each finger against the cube before this setpoint.
        closed = self._command_gripper(0.010)
        if closed.status != "SUCCESS":
            self._allow_grasp_contacts(obj, False)
            return closed
        if not self._set_gazebo_attachment(obj, True):
            self._allow_grasp_contacts(obj, False)
            return SkillResult("FAILED", f"Gazebo could not physically attach {obj}")
        self.held_object = obj
        self._set_attached_box(obj, True)
        if not self._allow_grasp_contacts(obj, False):
            return SkillResult("FAILED", "Object is physically held but collision settings did not reset")
        # A supported cube touches the table at the start of the lift. Permit
        # only this pair during the strictly upward, fixed-x/y movement.
        if not self._allow_contacts(obj, ["worktable"], True):
            return SkillResult("FAILED", "Could not allow pickup support contact")
        result = self._cartesian_raise(x, y, z)
        if not self._allow_contacts(obj, ["worktable"], False):
            return SkillResult("FAILED", "Could not restore table collision checking after lift")
        return result if result.status != "SUCCESS" else SkillResult("SUCCESS", f"picked {obj}")

    def place(self, obj, zone):
        if obj not in VALID_OBJECTS:
            return SkillResult("INVALID_OBJECT")
        if zone not in self.destinations:
            return SkillResult("INVALID_ZONE")
        if self.held_object != obj:
            return SkillResult("FAILED", f"{obj} is not held")
        state = self._observe_before_place(obj)
        target = self.destinations[zone]
        if any(math.dist(p[:2], target[:2]) < 0.12
               for name, p in state.objects.items() if name != obj):
            return SkillResult("FAILED", f"Destination {zone} is occupied")
        x, y, center_z = target
        z = center_z + 0.012  # Physical release 12 mm above tabletop.
        # Collision checking stays enabled throughout the descent.
        result = self._move_above_then_lower(x, y, z)
        if result.status != "SUCCESS":
            return result
        opened = self._command_gripper(0.0)
        if opened.status != "SUCCESS":
            return opened
        if not self._set_gazebo_attachment(obj, False):
            return SkillResult("FAILED", f"Gazebo could not physically release {obj}")
        self.held_object = None
        self._set_attached_box(obj, False, (x, y, z))
        if not self._allow_grasp_contacts(obj, True):
            return SkillResult("FAILED", "Object released, but gripper exit contact was not enabled")
        result = self._cartesian_raise(x, y, z)
        contacts_reset = self._allow_grasp_contacts(obj, False)
        if not contacts_reset:
            return SkillResult("FAILED", "Object released, but gripper contact settings did not reset")
        if result.status != "SUCCESS":
            return result
        # Move out of the overhead camera's view before checking the release.
        result = self.home()
        if result.status != "SUCCESS":
            return result
        observed = self.refresh_scene()
        actual = observed.objects[obj]
        if math.dist(actual[:2], target[:2]) > 0.025:
            return SkillResult("FAILED", f"Camera did not confirm {obj} at {zone}: {actual}")
        self.node.get_logger().info(f"CAMERA VERIFIED: {obj} at {zone}: {actual}")
        return SkillResult("SUCCESS", f"placed {obj} in {zone}")

    def execute(self, step):
        if step["skill"] == "detect_objects":
            self.refresh_scene()
            return SkillResult("SUCCESS", "five blocks detected by RGB camera")
        if step["skill"] == "check_zone":
            state = self.refresh_scene()
            occupants = state.occupants(step["zone"])
            return SkillResult("FAILED" if occupants else "SUCCESS", f"occupants={occupants}")
        if step["skill"] == "home":
            return self.home()
        if step["skill"] == "pick":
            return self.pick(step["object"])
        if step["skill"] == "place":
            return self.place(step["object"], step["zone"])
        return SkillResult("INVALID_SKILL")
