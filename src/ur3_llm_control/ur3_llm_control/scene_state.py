"""Camera-derived state and deterministic occupied-zone resolution (no ROS)."""
import math
from dataclasses import dataclass
from .task_validator import VALID_OBJECTS


class SceneError(RuntimeError):
    pass


@dataclass
class SceneState:
    objects: dict
    zones: dict
    zone_half_size: float = 0.055

    def occupants(self, zone):
        x, y, _ = self.zones[zone]
        # Include objects whose footprint overlaps a zone.
        return [name for name, p in self.objects.items()
                if abs(p[0] - x) < self.zone_half_size + 0.03
                and abs(p[1] - y) < self.zone_half_size + 0.03]

    def summary(self):
        complete = set(self.objects) == VALID_OBJECTS
        return {"objects": self.objects,
                "zones": {zone: self.occupants(zone) if complete or self.occupants(zone) else None
                          for zone in self.zones}}


def find_free_position(state, bounds, near):
    """Search observed free tabletop with finger clearance and zone exclusion."""
    xmin, xmax, ymin, ymax = bounds
    candidates = []
    for ix in range(int(round((xmax - xmin) / 0.025)) + 1):
        for iy in range(int(round((ymax - ymin) / 0.025)) + 1):
            x, y = xmin + ix * 0.025, ymin + iy * 0.025
            if any(math.hypot(x - p[0], y - p[1]) < 0.12 for p in state.objects.values()):
                continue
            if any(abs(x - p[0]) < state.zone_half_size + 0.055
                   and abs(y - p[1]) < state.zone_half_size + 0.055
                   for p in state.zones.values()):
                continue
            candidates.append((math.hypot(x - near[0], y - near[1]), x, y))
    if not candidates:
        raise SceneError("No observed free position with gripper clearance")
    _, x, y = min(candidates)
    return (x, y, next(iter(state.zones.values()))[2])


def resolve_occupied_zones(plan, observed, bounds):
    """Expand validated LLM pairs into checked skills using a virtual state."""
    if set(observed.objects) != VALID_OBJECTS:
        raise SceneError("All five blocks must be visible before resolving the plan")
    state = SceneState(dict(observed.objects), observed.zones, observed.zone_half_size)
    expanded, buffers = [{"skill": "detect_objects"}], {}
    assignments = {}
    for index in range(0, len(plan) - 1, 2):
        obj, zone = plan[index]["object"], plan[index + 1]["zone"]
        if zone in assignments and assignments[zone] != obj:
            raise SceneError(f"Conflicting final assignments for {zone}")
        assignments[zone] = obj
        blockers = [name for name in state.occupants(zone) if name != obj]
        if not blockers and state.occupants(zone) == [obj]:
            continue
        for blocker in blockers:
            position = find_free_position(state, bounds, state.objects[blocker])
            destination = f"temporary_{len(buffers) + 1}"
            buffers[destination] = position
            expanded.extend([
                {"skill": "pick", "object": blocker},
                {"skill": "place", "object": blocker, "zone": destination}])
            state.objects[blocker] = position
        expanded.extend([
            {"skill": "check_zone", "zone": zone},
            {"skill": "pick", "object": obj},
            {"skill": "place", "object": obj, "zone": zone}])
        state.objects[obj] = state.zones[zone]
    expanded.append({"skill": "home"})
    return expanded, buffers, assignments


def validate_execution_plan(plan, observed, buffers):
    """Validate expansion again, including holding state and occupancy."""
    state = SceneState(dict(observed.objects), observed.zones, observed.zone_half_size)
    held = None
    destinations = {**observed.zones, **buffers}
    if not plan or plan[0] != {"skill": "detect_objects"} or plan[-1] != {"skill": "home"}:
        raise SceneError("Execution plan must observe first and home last")
    for i, step in enumerate(plan):
        skill = step.get("skill")
        if skill == "detect_objects" and i == 0 and set(step) == {"skill"}:
            continue
        if skill == "home" and i == len(plan) - 1 and set(step) == {"skill"} and held is None:
            continue
        if skill == "check_zone" and set(step) == {"skill", "zone"}:
            if step["zone"] not in state.zones or state.occupants(step["zone"]):
                raise SceneError("Zone precondition failed")
            continue
        obj = step.get("object")
        if skill == "pick" and set(step) == {"skill", "object"}:
            if held is not None or obj not in state.objects:
                raise SceneError("Invalid pick precondition")
            held = obj
            del state.objects[obj]
            continue
        if skill == "place" and set(step) == {"skill", "object", "zone"}:
            dest = step["zone"]
            if held != obj or dest not in destinations:
                raise SceneError("Invalid place precondition")
            p = destinations[dest]
            if any(math.dist(p[:2], q[:2]) < 0.12 for q in state.objects.values()):
                raise SceneError("Place destination is occupied or lacks clearance")
            state.objects[obj], held = p, None
            continue
        raise SceneError(f"Invalid execution step {i + 1}")
