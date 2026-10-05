"""Interactive terminal for submitting language tasks to a running simulation."""
import argparse
import json
import time
import uuid
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.utilities import remove_ros_args
from std_msgs.msg import String
from .command_session import TERMINAL_STATES
from .command_transport import COMMAND_TOPIC, STATUS_TOPIC, status_qos


class CommandClient(Node):
    def __init__(self):
        super().__init__("llm_command_client_" + uuid.uuid4().hex[:8])
        self.publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.subscription = self.create_subscription(String, STATUS_TOPIC, self._status, status_qos())
        self.active_id, self.result, self.availability = None, None, None
        self.last_progress = None

    def _status(self, message):
        try:
            status = json.loads(message.data)
            if not isinstance(status, dict):
                return
        except ValueError:
            return
        if not status.get("task_id") and status.get("state") in {"READY", "BUSY", "FAULT"}:
            self.availability = status
        if self.active_id is None or status.get("task_id") != self.active_id:
            return
        state = status.get("state")
        progress = (state, status.get("step"))
        if progress != self.last_progress:
            self.last_progress = progress
            if state == "VALIDATED":
                print(f"[{self.active_id[:8]}] VALIDATED ({status.get('planner_source')})", flush=True)
                for index, step in enumerate(status.get("execution_plan", []), 1):
                    print(f"  {index}. {json.dumps(step, ensure_ascii=False)}", flush=True)
            elif state == "EXECUTING":
                print(f"[{self.active_id[:8]}] EXECUTING {status.get('step')}/{status.get('total_steps')}: "
                      + json.dumps(status.get("skill"), ensure_ascii=False), flush=True)
            else:
                print(f"[{self.active_id[:8]}] {state}", flush=True)
        if state in TERMINAL_STATES:
            self.result = status
            if status.get("error"):
                print("  " + status["error"], flush=True)
            if status.get("evidence_file"):
                print("  Evidence: " + status["evidence_file"], flush=True)

    def wait_connected(self, timeout=90.0):
        print("Waiting for continuous command server...", flush=True)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            count = self.publisher.get_subscription_count()
            if count > 1:
                raise RuntimeError("Multiple command servers detected; stop extra launches or use separate ROS_DOMAIN_ID")
            if count == 1 and self.availability is not None:
                return
        raise RuntimeError("No command server; launch llm_robot.launch.py with continuous:=true in the same ROS domain")

    def send(self, command, timeout=300.0):
        self.active_id, self.result, self.last_progress = uuid.uuid4().hex, None, None
        if self.publisher.get_subscription_count() != 1:
            raise RuntimeError("Expected exactly one command server; command was not sent")
        self.publisher.publish(String(data=json.dumps({"task_id": self.active_id, "command": command}, ensure_ascii=False)))
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.result is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if not self.publisher.get_subscription_count() and self.result is None:
                raise RuntimeError("Command server disconnected. Inspect robot state before submitting again")
        if self.result is None:
            raise RuntimeError("No terminal result within timeout. Task may still be running; inspect /llm_status before submitting again")
        return self.result["state"] in {"SUCCEEDED", "PLANNED"}


def main(args=None):
    parser = argparse.ArgumentParser(description="Send continuous natural-language commands to the UR3 LLM planner")
    parser.add_argument("--command", help="Send one command and exit; omit for an interactive prompt")
    parser.add_argument("--timeout", type=float, default=300.0, help="Seconds to wait for a task result")
    parser.add_argument("--connect-timeout", type=float, default=90.0)
    options = parser.parse_args(remove_ros_args(args=args)[1:])
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = CommandClient()
    exit_code = 0
    try:
        node.wait_connected(options.connect_timeout)
        print("Connected. Enter a command; exit/quit closes this terminal.", flush=True)
        if options.command is not None:
            exit_code = 0 if node.send(options.command, options.timeout) else 1
        else:
            while rclpy.ok():
                command = input("LLM > ").strip()
                if command.lower() in {"exit", "quit"}:
                    break
                if command:
                    node.send(command, options.timeout)
    except (EOFError, KeyboardInterrupt):
        print("\nCommand client closed. An active robot task continues in the launch process.", flush=True)
    except RuntimeError as exc:
        print(str(exc), flush=True)
        exit_code = 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if exit_code:
        raise SystemExit(exit_code)
