import copy
import math
from pathlib import Path
import pytest
import yaml
import cv2
import numpy as np
from ur3_llm_control.task_validator import PlanValidator, VALID_OBJECTS
from ur3_llm_control.scene_state import (SceneError, SceneState, resolve_occupied_zones,
                                         validate_execution_plan, find_free_position)
from ur3_llm_control.perception import detect_blocks

CONFIG = yaml.safe_load((Path(__file__).parents[1] / 'config/scene.yaml').read_text())
POSITIONS = {'red_cube': (0.34, -0.22, 0.5), 'yellow_cube': (0.34, 0, 0.5),
             'blue_cube': (0.54, 0, 0.5), 'green_cube': (0.34, 0.22, 0.5),
             'purple_cube': (0.54, 0.24, 0.5)}
PLAN = [{'skill': 'pick', 'object': 'red_cube'},
        {'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'}, {'skill': 'home'}]


def test_blocked_zone_cleared_before_requested_pick():
    state = SceneState(POSITIONS, CONFIG['zones'])
    steps, buffers, assignments = resolve_occupied_zones(PLAN, state, CONFIG['buffer_bounds'])
    assert steps[1] == {'skill': 'pick', 'object': 'blue_cube'}
    assert steps[2]['zone'] in buffers
    assert steps[3] == {'skill': 'check_zone', 'zone': 'zone_b'}
    assert steps[4] == PLAN[0]
    validate_execution_plan(steps, state, buffers)
    assert assignments == {'zone_b': 'red_cube'}
    assert all(math.dist(buffers['temporary_1'][:2], p[:2]) >= 0.12 for p in POSITIONS.values())


@pytest.mark.parametrize('bad', [
    [{'skill': 'place', 'object': 'red_cube', 'zone': 'zone_b'}, {'skill': 'home'}],
    [PLAN[0], {'skill': 'pick', 'object': 'blue_cube'}, PLAN[2]],
    [PLAN[0], PLAN[2]], [PLAN[2], *PLAN], PLAN[:-1],
    [{'skill': 'pick', 'object': ['red_cube']}],
    [{'skill': ['pick']}],
    [PLAN[0], {**PLAN[1], 'pose': [1, 2, 3]}, PLAN[2]],
    [PLAN[0], {**PLAN[1], 'zone': 'temporary_1'}, PLAN[2]],
    [{'skill': 'joint_trajectory', 'joints': [1]}],
])
def test_llm_schema_and_preconditions_rejected(bad):
    assert not PlanValidator().validate({'plan': bad}).valid


def test_unknown_root_and_five_objects():
    assert not PlanValidator().validate({'plan': PLAN, 'trajectory': []}).valid
    for obj in VALID_OBJECTS:
        plan = copy.deepcopy(PLAN)
        plan[0]['object'] = plan[1]['object'] = obj
        assert PlanValidator().validate({'plan': plan}).valid


def test_missing_camera_object_and_full_table_stop():
    state = SceneState({k: v for k, v in POSITIONS.items() if k != 'green_cube'}, CONFIG['zones'])
    with pytest.raises(SceneError):
        resolve_occupied_zones(PLAN, state, CONFIG['buffer_bounds'])
    with pytest.raises(SceneError):
        find_free_position(SceneState(POSITIONS, CONFIG['zones']), [0.34, 0.34, 0, 0], (0.34, 0, 0.5))


def test_occupied_destination_execution_rejected():
    state = SceneState(POSITIONS, CONFIG['zones'])
    with pytest.raises(SceneError):
        validate_execution_plan([{'skill': 'detect_objects'}, *PLAN], state, {})


def test_already_correct_noop_and_conflicting_assignments():
    state = SceneState({**POSITIONS, 'red_cube': (0.54, -0.24, 0.5)}, CONFIG['zones'])
    plan = copy.deepcopy(PLAN)
    plan[1]['zone'] = 'zone_a'
    steps, buffers, _ = resolve_occupied_zones(plan, state, CONFIG['buffer_bounds'])
    assert steps == [{'skill': 'detect_objects'}, {'skill': 'home'}]
    duplicate = [*PLAN[:-1], {'skill': 'pick', 'object': 'green_cube'},
                 {'skill': 'place', 'object': 'green_cube', 'zone': 'zone_b'}, PLAN[-1]]
    with pytest.raises(SceneError):
        resolve_occupied_zones(duplicate, SceneState(POSITIONS, CONFIG['zones']), CONFIG['buffer_bounds'])


def render_blocks(positions):
    image = np.full((800, 800, 3), 70, np.uint8)
    focal = 800 / (2 * math.tan(CONFIG['camera']['horizontal_fov'] / 2))
    depth = 1.65 - 0.53
    half = int(round(focal * 0.03 / depth))
    colors = {'red_cube': (0, 0, 230), 'yellow_cube': (0, 230, 230),
              'blue_cube': (230, 40, 0), 'green_cube': (0, 230, 0), 'purple_cube': (230, 0, 200)}
    for name, (x, y, _) in positions.items():
        u = int(round(399.5 - y * focal / depth))
        v = int(round(399.5 - (x - 0.47) * focal / depth))
        cv2.rectangle(image, (u-half, v-half), (u+half, v+half), colors[name], -1)
    return image


def test_rgb_perception_tracks_changed_positions():
    changed = {**POSITIONS, 'blue_cube': (0.425, -0.125, 0.5)}
    for positions in (POSITIONS, changed):
        state = detect_blocks(render_blocks(positions), CONFIG)
        assert set(state.objects) == VALID_OBJECTS
        assert all(math.dist(state.objects[name], p) < 0.003 for name, p in positions.items())
    assert detect_blocks(render_blocks(POSITIONS), CONFIG).occupants('zone_b') == ['blue_cube']
    assert detect_blocks(render_blocks(changed), CONFIG).occupants('zone_b') == []


def test_blank_or_ambiguous_image_does_not_invent_objects():
    assert not detect_blocks(np.zeros((800, 800, 3), np.uint8), CONFIG).objects
    image = render_blocks(POSITIONS)
    other = render_blocks({'red_cube': (0.34, -0.08, 0.5)})
    image[other[:, :, 2] > 200] = (0, 0, 230)
    assert 'red_cube' not in detect_blocks(image, CONFIG).objects


def test_student_mapping_with_extra_cube_blocking_zone_c():
    plan = []
    for obj, zone in [('red_cube', 'zone_a'), ('blue_cube', 'zone_b'), ('yellow_cube', 'zone_c')]:
        plan.extend([{'skill': 'pick', 'object': obj},
                     {'skill': 'place', 'object': obj, 'zone': zone}])
    plan.append({'skill': 'home'})
    state = SceneState(POSITIONS, CONFIG['zones'])
    expanded, buffers, assignments = resolve_occupied_zones(plan, state, CONFIG['buffer_bounds'])
    validate_execution_plan(expanded, state, buffers)
    assert {'skill': 'pick', 'object': 'purple_cube'} in expanded
    assert {'skill': 'pick', 'object': 'blue_cube'} not in expanded
    assert assignments == {'zone_a': 'red_cube', 'zone_b': 'blue_cube', 'zone_c': 'yellow_cube'}


def test_occluded_object_makes_empty_zone_state_unknown():
    partial = {name: pose for name, pose in POSITIONS.items() if name != 'blue_cube'}
    summary = detect_blocks(render_blocks(partial), CONFIG).summary()
    assert summary['zones']['zone_b'] is None
    assert summary['zones']['zone_c'] == ['purple_cube']
    assert detect_blocks(render_blocks(POSITIONS), CONFIG).summary()['zones']['zone_a'] == []
