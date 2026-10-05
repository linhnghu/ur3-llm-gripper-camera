"""Natural language -> validated LLM plan -> camera-aware skills -> MoveIt."""
import json
import threading
import queue
import signal
from pathlib import Path
import yaml
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from .llm_planner import LLMPlanner, PlannerError
from .robot_skills import RobotSkills
from .task_validator import PlanValidator
from .perception import CameraObserver
from .scene_state import SceneError, resolve_occupied_zones, validate_execution_plan
from .command_session import CommandSession
from .command_transport import COMMAND_TOPIC, STATUS_TOPIC, status_qos


class LLMTaskNode(Node):
    def __init__(self):
        super().__init__("llm_task_planner")
        for name, default in (
            ("robot_description", ""),
            ("command", ""), ("api_key", ""),
            ("endpoint", "https://9router.com/v1/chat/completions"),
            ("model", "gpt-4o-mini"), ("execute", True),
            ("group_name", "ur_manipulator"), ("student_id", "23020749"),
            ("demo_plan", False), ("record_path", ""), ("evidence_path", ""),
            ("continuous", False),
        ):
            self.declare_parameter(name, default)
        config_path = Path(get_package_share_directory("ur3_llm_control")) / "config/scene.yaml"
        self.config = yaml.safe_load(config_path.read_text())
        self.observer = CameraObserver(self, self.config, self.get_parameter("record_path").value)
        self.planner = LLMPlanner(
            endpoint=self.get_parameter("endpoint").value,
            model=self.get_parameter("model").value,
            api_key=self.get_parameter("api_key").value,
            student_id=self.get_parameter("student_id").value)
        self.validator = PlanValidator()
        self.evidence = {"success": False, "steps": []}
        self.skills = None
        self.task_id = None
        self.execution_started = False
        self.session = CommandSession()
        self.commands = queue.Queue(maxsize=1)
        self.status_publisher = self.create_publisher(String, STATUS_TOPIC, status_qos())
        if self.get_parameter("continuous").value:
            self.command_subscription = self.create_subscription(String, COMMAND_TOPIC, self._on_command, 10)
            self.availability_timer = self.create_timer(1.0, self._publish_availability)

    def _publish_status(self, status):
        self.status_publisher.publish(String(data=json.dumps(status, ensure_ascii=False)))

    def _publish_availability(self):
        self._publish_status(self.session.availability())

    def _on_command(self, message):
        command, task_id = message.data, None
        if command.lstrip().startswith("{"):
            try:
                data = json.loads(command)
                if not isinstance(data, dict) or set(data) != {"task_id", "command"}:
                    raise ValueError("Expected JSON with exactly task_id and command")
                task_id, command = data["task_id"], data["command"]
            except (ValueError, TypeError) as exc:
                self._publish_status({"task_id": "", "state": "REJECTED", "error": str(exc)})
                return
        task, status = self.session.submit(command, task_id)
        self._publish_status(status)
        if task:
            self.commands.put_nowait(task)

    def _task_status(self, state, **details):
        if self.task_id:
            self._publish_status(self.session.update(self.task_id, state, **details))

    def serve_commands(self):
        self.get_logger().info(f"CONTINUOUS READY: receive commands on {COMMAND_TOPIC}; use ros2 run ur3_llm_control llm_command")
        initial_command = self.get_parameter("command").value
        if initial_command:
            self._on_command(String(data=initial_command))
        self._publish_availability()
        while rclpy.ok():
            try:
                task = self.commands.get(timeout=0.2)
            except queue.Empty:
                continue
            self.task_id = task.task_id
            try:
                succeeded = self.run_task(task.command)
            except Exception as exc:
                self.evidence["error"] = str(exc)
                self.get_logger().error(f"Task failed unexpectedly: {exc}")
                succeeded = False
            fault = self.evidence.get("error", "Execution failed") if not succeeded and self.execution_started else ""
            state = ("SUCCEEDED" if self.evidence.get("success") else "PLANNED") if succeeded else "FAILED"
            details = {"error": self.evidence.get("error", ""), "held_object": self.observer.held_object}
            if "evidence_file" in self.evidence:
                details["evidence_file"] = self.evidence["evidence_file"]
            self._publish_status(self.session.finish(task.task_id, state, fault=fault, **details))
            self.task_id = None
            self._publish_availability()

    @staticmethod
    def step_name(step):
        args = [value for key, value in step.items() if key != "skill"]
        return f"{step['skill']}({', '.join(args)})"

    def run_task(self, command):
        self.evidence = {"success": False, "steps": [], "task_id": self.task_id}
        self.execution_started = False
        self._task_status("PLANNING")
        self.get_logger().info(f"USER COMMAND: {command}")
        self.evidence["command"] = command
        try:
            if self.observer.held_object or (self.skills and self.skills.held_object):
                raise SceneError("Robot still holds an object; inspect simulation before restarting")
            initial = self.observer.snapshot(timeout=40.0)
            self.evidence["initial_camera_state"] = initial.summary()
            self.get_logger().info("CAMERA STATE: " + json.dumps(initial.summary()))
            if self.get_parameter("demo_plan").value:
                # Explicit, fixed regression fixture. This is not an LLM demo.
                if command.strip().lower() != "put the red cube in zone b.":
                    raise PlannerError("demo_plan only supports: Put the red cube in zone B.")
                self.get_logger().warn("OFFLINE FIXTURE: LLM request bypassed for simulation regression")
                payload = {"plan": [{"skill": "pick", "object": "red_cube"},
                                    {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                                    {"skill": "home"}]}
            else:
                payload = self.planner.create_plan(command, initial.summary())
            self.evidence["planner_source"] = "offline_fixture" if self.get_parameter("demo_plan").value else "LLM"
            checked = self.validator.validate(payload)
            if not checked.valid:
                raise SceneError("PLAN_REJECTED: " + checked.error)
            self.get_logger().info(f"SYMBOLIC PLAN [{self.evidence['planner_source']}]: "
                                   + json.dumps({"plan": checked.plan}))
            # Refresh after the network request; no movement uses a stale snapshot.
            observed = self.observer.snapshot()
            plan, buffers, assignments = resolve_occupied_zones(
                checked.plan, observed, self.config["buffer_bounds"])
            validate_execution_plan(plan, observed, buffers)
            self.evidence.update(llm_plan=checked.plan, execution_plan=plan, buffers=buffers)
            self._task_status("VALIDATED", execution_plan=plan, planner_source=self.evidence["planner_source"])
            self.get_logger().info("VALIDATED EXECUTION PLAN:\n" + "\n".join(
                f"{i}. {self.step_name(step)}" for i, step in enumerate(plan, 1)))
            self.get_logger().info("CAMERA-DERIVED BUFFERS: " + json.dumps(buffers))
            if not self.get_parameter("execute").value:
                self.get_logger().info("EXECUTION SKIPPED (execute=false)")
                self.evidence["planning_success"] = True
                return True
            self.execution_started = True
            if self.skills is None:
                self.skills = RobotSkills(self, self.observer, self.config, buffers,
                                          self.get_parameter("group_name").value)
            self.skills.destinations = {**self.config["zones"], **buffers}
            for index, step in enumerate(plan, 1):
                self._task_status("EXECUTING", step=index, total_steps=len(plan), skill=step)
                result = self.skills.execute(step)
                self.evidence["steps"].append({**step, "status": result.status, "detail": result.detail})
                self.get_logger().info(f"{self.step_name(step)}: {result.status} {result.detail}")
                if result.status != "SUCCESS":
                    raise SceneError(f"TASK FAILED at step {index}: {result.detail}")
            final = self.observer.snapshot()
            for zone, obj in assignments.items():
                if final.occupants(zone) != [obj]:
                    raise SceneError(f"Final camera postcondition failed: {zone} must contain {obj}")
            self.evidence.update(success=True, final_camera_state=final.summary())
            self.get_logger().info("TASK SUCCESS — verified by camera")
            return True
        except (PlannerError, SceneError, RuntimeError) as exc:
            self.evidence["error"] = str(exc)
            self.get_logger().error(str(exc))
            return False
        finally:
            path = self.get_parameter("evidence_path").value
            if path:
                target = Path(path)
                if self.task_id:
                    target = target.with_name(f"{target.stem}_{self.task_id}{target.suffix or '.json'}")
                try:
                    self.evidence["evidence_file"] = str(target)
                    target.write_text(json.dumps(self.evidence, indent=2) + "\n")
                except OSError as exc:
                    self.evidence.pop("evidence_file", None)
                    self.get_logger().error(f"Could not save task evidence: {exc}")


def main(args=None):
    # Stop the executor before shutting down its context. Automatic rclpy
    # shutdown on SIGINT can invalidate a background executor's wait set.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)

    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, interrupt)
    signal.signal(signal.SIGTERM, interrupt)
    node = LLMTaskNode()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=False)
    thread.start()
    succeeded = False
    try:
        if node.get_parameter("continuous").value:
            node.serve_commands()
            succeeded = True
        else:
            command = node.get_parameter("command").value or input("USER COMMAND: ")
            succeeded = node.run_task(command)
    except (EOFError, KeyboardInterrupt):
        succeeded = node.get_parameter("continuous").value
    finally:
        executor.shutdown()
        thread.join(timeout=3.0)
        node.observer.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if not succeeded:
        raise SystemExit(1)
