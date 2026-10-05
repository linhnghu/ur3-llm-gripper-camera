"""Thread-safe admission of one language task at a time, independent of ROS."""
from collections import OrderedDict
from dataclasses import dataclass
import re
import threading
import uuid

TERMINAL_STATES = {"SUCCEEDED", "PLANNED", "FAILED", "REJECTED"}


@dataclass(frozen=True)
class CommandTask:
    task_id: str
    command: str


class CommandSession:
    def __init__(self, history_limit=128):
        self.lock = threading.Lock()
        self.history = OrderedDict()
        self.history_limit = history_limit
        self.active_id = None
        self.fault = ""

    def submit(self, command, task_id=None):
        task_id = uuid.uuid4().hex if task_id is None else task_id
        status = {"task_id": task_id if isinstance(task_id, str) else "",
                  "state": "REJECTED"}
        if not isinstance(task_id, str) or not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", task_id):
            return None, {**status, "error": "Invalid task_id; use 1-64 letters, digits, _ or -"}
        if not isinstance(command, str) or not command.strip() or len(command) > 1000:
            return None, {**status, "error": "Command must contain 1-1000 characters"}
        command = command.strip()
        with self.lock:
            if task_id in self.history:
                previous = self.history[task_id]
                if previous["command"] != command:
                    return None, {**status, "error": "task_id already belongs to another command"}
                return None, dict(previous)  # Never run a duplicate request twice.
            status["command"] = command
            if self.fault:
                status["error"] = "Execution fault; restart after inspecting robot: " + self.fault
            elif self.active_id:
                status.update(error="Robot is busy; submit again when READY", active_task_id=self.active_id)
            else:
                status["state"] = "ACCEPTED"
                self.active_id = task_id
            self.history[task_id] = dict(status)
            self._trim()
            task = CommandTask(task_id, command) if status["state"] == "ACCEPTED" else None
            return task, status

    def update(self, task_id, state, **details):
        with self.lock:
            if task_id != self.active_id or state in TERMINAL_STATES:
                raise ValueError("Only the active task may transition before completion")
            status = {**self.history[task_id], **details, "state": state}
            self.history[task_id] = status
            return dict(status)

    def finish(self, task_id, state, fault="", **details):
        with self.lock:
            if task_id != self.active_id or state not in TERMINAL_STATES:
                raise ValueError("Only the active task may finish")
            status = {**self.history[task_id], **details, "state": state}
            self.history[task_id] = status
            self.active_id = None
            self.fault = fault
            self._trim()
            return dict(status)

    def availability(self):
        with self.lock:
            return {"task_id": "", "state": "FAULT" if self.fault else "BUSY" if self.active_id else "READY",
                    "active_task_id": self.active_id, "error": self.fault}

    def _trim(self):
        while len(self.history) > self.history_limit:
            oldest = next(key for key in self.history if key != self.active_id)
            del self.history[oldest]
