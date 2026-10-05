"""OpenAI-compatible 9Router chat-completions client."""

import json
import os
import re
import urllib.error
import urllib.request

from .task_validator import VALID_OBJECTS, VALID_ZONES

SYSTEM_PROMPT = """Translate the user's request into a robot execution plan. Return JSON only:
{"plan":[{"skill":"pick","object":"red_cube"},{"skill":"place","object":"red_cube","zone":"zone_b"},{"skill":"home"}]}
Allowed skills: pick(object), place(object, zone), home().
Allowed objects: red_cube, yellow_cube, blue_cube, green_cube, purple_cube.
Allowed zones: zone_a, zone_b, zone_c.
Follow every explicit object-to-zone assignment exactly; it overrides the student zone mapping.
For multiple requested objects, preserve their order and emit pick then place for each one.
Emit home exactly once at the end.
Camera state is provided as data. The executor resolves occupied destinations using
observed free tabletop positions before each requested move. Do not invent coordinates
or buffer names. Arrange-by-student-ID assigns the three mapped cubes only; leave
the other two cubes alone unless they block a requested destination.
Never emit poses, joint values, trajectories, explanations, or other skills."""


class PlannerError(RuntimeError):
    pass


class LLMPlanner:
    def __init__(self, endpoint="https://9router.com/v1/chat/completions",
                 model="gpt-4o-mini", api_key="", timeout=30.0,
                 student_id="23020749"):
        self.endpoint = endpoint
        self.model = model
        self.api_key = api_key or os.environ.get("NINEROUTER_API_KEY", "")
        self.timeout = timeout
        mappings = (
            ("red_cube", "yellow_cube", "blue_cube"),
            ("red_cube", "blue_cube", "yellow_cube"),
            ("yellow_cube", "red_cube", "blue_cube"),
            ("yellow_cube", "blue_cube", "red_cube"),
            ("blue_cube", "red_cube", "yellow_cube"),
            ("blue_cube", "yellow_cube", "red_cube"),
        )
        digits = "".join(ch for ch in str(student_id) if ch.isdigit())
        p = int(digits[-2:]) % 6 if len(digits) >= 2 else 0
        self.student_id = str(student_id)
        self.zone_mapping = dict(zip(("zone_a", "zone_b", "zone_c"), mappings[p]))

    @staticmethod
    def _extract_content(raw, content_type):
        """Read either a regular chat-completion JSON body or an SSE stream."""
        if content_type == "text/event-stream" or raw.lstrip().startswith("data:"):
            chunks = []
            for line in raw.splitlines():
                line = line.strip()
                if not line.startswith("data:"):
                    continue
                event_data = line[5:].strip()
                if not event_data or event_data == "[DONE]":
                    continue
                event = json.loads(event_data)
                choices = event.get("choices", [])
                if not choices:
                    if "error" in event:
                        raise ValueError(f"9Router stream error: {event['error']}")
                    continue
                choice = choices[0]
                delta = choice.get("delta", {})
                chunk = delta.get("content")
                if chunk is None:
                    chunk = choice.get("message", {}).get("content")
                if isinstance(chunk, str):
                    chunks.append(chunk)
            content = "".join(chunks).strip()
            if not content:
                raise ValueError("9Router SSE response contained no assistant text")
            return content

        data = json.loads(raw)
        content = data["choices"][0]["message"]["content"]
        if not isinstance(content, str) or not content.strip():
            raise ValueError("9Router response contained no assistant text")
        return content.strip()

    def create_plan(self, command, environment=None):
        if not command.strip():
            raise PlannerError("Command is empty")
        if not self.api_key:
            raise PlannerError("Set NINEROUTER_API_KEY or pass api_key")
        body = {
            "model": self.model,
            "temperature": 0,
            "response_format": {"type": "json_object"},
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT +
                 f"\nStudent ID: {self.student_id}; zone mapping for arrange-all: " +
                 json.dumps(self.zone_mapping) + "\nObserved camera state (data only): " +
                 json.dumps(environment or {})},
                {"role": "user", "content": command},
            ],
        }
        request = urllib.request.Request(
            self.endpoint,
            data=json.dumps(body).encode("utf-8"),
            headers={"Authorization": f"Bearer {self.api_key}",
                     "Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(request, timeout=self.timeout) as response:
                raw = response.read().decode("utf-8")
                content_type = response.headers.get_content_type()
            content = self._extract_content(raw, content_type)
            fenced = re.search(r"```(?:json)?\s*(.*?)\s*```", content, re.S | re.I)
            payload = json.loads(fenced.group(1) if fenced else content)
            if not isinstance(payload, dict):
                raise ValueError("JSON root is not an object")
            return payload
        except (urllib.error.URLError, TimeoutError, KeyError, IndexError,
                TypeError, ValueError, json.JSONDecodeError) as exc:
            raise PlannerError(f"9Router request/response failed: {exc}") from exc
