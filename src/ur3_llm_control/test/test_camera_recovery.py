import threading
import time
from types import SimpleNamespace
import pytest
from ur3_llm_control.perception import CameraObserver, CameraStateError
from ur3_llm_control.robot_skills import RobotSkills, SkillResult
from ur3_llm_control.scene_state import SceneState, SceneError
from ur3_llm_control.task_validator import VALID_OBJECTS


def test_snapshot_distinguishes_missing_object_from_stale_camera():
    partial = SceneState({name: (0.4, 0, 0.5) for name in VALID_OBJECTS - {"yellow_cube"}}, {})
    observer = SimpleNamespace(condition=threading.Condition(), sequence=1,
                               frames=[(1, time.monotonic(), partial, 1.0)], last_error="RGB image processed")
    with pytest.raises(CameraStateError) as error:
        CameraObserver.snapshot(observer, timeout=0, after=0)
    assert error.value.reason == "missing_objects"
    assert error.value.missing_objects == {"yellow_cube"}
    observer.frames[0] = (1, time.monotonic() - 10, partial, 1.0)
    with pytest.raises(CameraStateError) as error:
        CameraObserver.snapshot(observer, timeout=0, after=0)
    assert error.value.reason == "no_fresh_images"


def make_skills():
    skills = RobotSkills.__new__(RobotSkills)
    skills.held_object = "red_cube"
    skills.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(warn=lambda _: None, info=lambda _: None))
    return skills


def test_camera_recovery_requires_new_observation_without_releasing_grasp():
    skills = make_skills()
    observed = SceneState({name: (0.4, 0, 0.5) for name in VALID_OBJECTS - {"red_cube"}}, {})
    calls = []

    def refresh(required, timeout=20.0):
        calls.append((required, timeout))
        if len(calls) == 1:
            raise CameraStateError("yellow hidden", "missing_objects", {"yellow_cube"})
        return observed

    moved = []
    skills.refresh_scene = refresh
    skills._move_to_observation_pose = lambda: (moved.append(True) or SkillResult("SUCCESS"))
    assert skills._observe_before_place("red_cube") is observed
    assert moved and len(calls) == 2
    assert all(required == VALID_OBJECTS - {"red_cube"} for required, _ in calls)
    assert skills.held_object == "red_cube"


@pytest.mark.parametrize("reason", ["no_fresh_images", "unstable_objects"])
def test_camera_outage_or_moving_objects_do_not_trigger_recovery_motion(reason):
    skills = make_skills()

    def refresh(*args, **kwargs):
        raise CameraStateError("No usable observation", reason, {"yellow_cube"})

    def must_not_move():
        pytest.fail("Camera outage must not cause robot movement")

    skills.refresh_scene = refresh
    skills._move_to_observation_pose = must_not_move
    with pytest.raises(CameraStateError):
        skills._observe_before_place("red_cube")


def test_failed_recovery_motion_stops_before_place():
    skills = make_skills()
    observations = []

    def refresh(*args, **kwargs):
        observations.append(True)
        raise CameraStateError("yellow hidden", "missing_objects", {"yellow_cube"})

    skills.refresh_scene = refresh
    skills._move_to_observation_pose = lambda: SkillResult("PLANNING_FAILED", "no collision-free path")
    with pytest.raises(SceneError, match="Camera view recovery motion failed"):
        skills._observe_before_place("red_cube")
    assert len(observations) == 1
    assert skills.held_object == "red_cube"


def test_missing_object_after_recovery_still_stops_without_inventing_pose():
    skills = make_skills()
    observations = []

    def refresh(*args, **kwargs):
        observations.append(True)
        raise CameraStateError("yellow still not visible", "missing_objects", {"yellow_cube"})

    skills.refresh_scene = refresh
    skills._move_to_observation_pose = lambda: SkillResult("SUCCESS")
    with pytest.raises(CameraStateError, match="yellow still not visible"):
        skills._observe_before_place("red_cube")
    assert len(observations) == 2
    assert skills.held_object == "red_cube"
