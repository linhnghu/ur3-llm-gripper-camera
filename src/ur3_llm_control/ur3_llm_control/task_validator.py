"""Allow-list validation for plans returned by the language model."""

from dataclasses import dataclass
from typing import Any, Dict, List, Set

VALID_OBJECTS = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"}
VALID_ZONES = {"zone_a", "zone_b", "zone_c"}
VALID_SKILLS = {"pick", "place", "home"}


@dataclass
class ValidationResult:
    valid: bool
    plan: List[Dict[str, str]]
    error: str = ""


class PlanValidator:
    def __init__(self, objects: Set[str] = None, zones: Set[str] = None):
        self.objects = set(objects or VALID_OBJECTS)
        self.zones = set(zones or VALID_ZONES)

    def validate(self, payload: Any) -> ValidationResult:
        if (not isinstance(payload, dict) or set(payload) != {"plan"}
                or not isinstance(payload.get("plan"), list)):
            return ValidationResult(False, [], "Expected JSON object with a plan array")
        result = []
        held = None
        if len(payload["plan"]) > 41:
            return ValidationResult(False, [], "Plan exceeds 41 steps")
        for i, step in enumerate(payload["plan"], 1):
            if not isinstance(step, dict):
                return ValidationResult(False, [], f"Step {i} must be an object")
            skill = step.get("skill")
            if not isinstance(skill, str) or skill not in VALID_SKILLS:
                return ValidationResult(False, [], f"Step {i}: invalid skill {skill!r}")
            if skill == "home":
                if set(step) != {"skill"}:
                    return ValidationResult(False, [], f"Step {i}: home takes no arguments")
                if held is not None or i != len(payload["plan"]):
                    return ValidationResult(False, [], "home must be last, with no object held")
                result.append({"skill": "home"})
            elif skill == "pick":
                obj = step.get("object")
                if not isinstance(obj, str) or obj not in self.objects:
                    return ValidationResult(False, [], f"Step {i}: invalid object {obj!r}")
                if set(step) != {"skill", "object"}:
                    return ValidationResult(False, [], f"Step {i}: pick has unexpected arguments")
                if held is not None:
                    return ValidationResult(False, [], f"Step {i}: already holding {held}")
                held = obj
                result.append({"skill": "pick", "object": obj})
            else:
                obj, zone = step.get("object"), step.get("zone")
                if not isinstance(obj, str) or obj not in self.objects:
                    return ValidationResult(False, [], f"Step {i}: invalid object {obj!r}")
                if not isinstance(zone, str) or zone not in self.zones:
                    return ValidationResult(False, [], f"Step {i}: invalid zone {zone!r}")
                if set(step) != {"skill", "object", "zone"}:
                    return ValidationResult(False, [], f"Step {i}: place has unexpected arguments")
                if held != obj:
                    return ValidationResult(False, [], f"Step {i}: {obj} is not held")
                held = None
                result.append({"skill": "place", "object": obj, "zone": zone})
        if not result:
            return ValidationResult(False, [], "Plan is empty")
        if held is not None or result[-1] != {"skill": "home"}:
            return ValidationResult(False, [], "Plan must finish with home and no held object")
        return ValidationResult(True, result)
