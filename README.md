# UR3e Simulation: MoveIt 2, Gripper, Camera and LLM Skill Planning

ROS 2 Humble workspace for simulating a UR3e in Gazebo, planning with MoveIt 2,
and executing language-described pick-and-place tasks. It also contains a
Cartesian path demo that draws the letter **L**.

> This repository is for simulation and coursework. Do not connect the task
> controller to a physical robot without adding hardware-specific safety
> checks, workspace limits, and an emergency-stop procedure.

## Bài 03: camera và zone bị chiếm

Xem [hướng dẫn chạy Bài 03](src/ur3_llm_control/README.md) và
[báo cáo thiết kế](src/ur3_llm_control/REPORT_BAI03.md). World mới có 5 cube, camera RGB
và physics grasp plugin kiểm tra tiếp xúc hai ngón. Zone B ban đầu có blue cube;
resolver tự chèn di chuyển blue tới buffer trước khi đặt red vào B.
Build thêm package `ur3_grasp_plugin` trước khi chạy launch LLM.

Có thể nhập lệnh liên tục trong cùng một phiên bằng `continuous:=true` và
`ros2 run ur3_llm_control llm_command`; xem
[hướng dẫn kết nối và nhập lệnh](src/ur3_llm_control/README.md#nhập-lệnh-liên-tục-khi-mô-phỏng-đang-chạy).

## What is included

- `src/ur3_draw_letter`: UR3e description/configuration, a two-finger simulated
  gripper, and a MoveIt Cartesian letter-drawing demo.
- `src/ur3_grasp_plugin`: contact-gated Gazebo physics grasp constraints.
- `src/ur3_llm_control`: RGB perception, occupied-zone resolver, LLM planner, validators, robot skills,
  Gazebo task world, and the combined simulation/MoveIt launch file.
- `src/ur_simulation_gz`: Gazebo simulation and MoveIt launch support for
  Universal Robots, included as source files in this repository.

The LLM is only asked to produce a small JSON plan using `pick`, `place`, and
`home`. The validator rejects unknown skills, objects, zones, and extra
arguments before execution. MoveIt plans arm motions; the gripper controller
commands the fingers; Gazebo simulates object contact and attachment.

For student ID `23020749`, the configured default mapping is:

| Zone | Object |
| --- | --- |
| A | Red cube |
| B | Blue cube |
| C | Yellow cube |

An explicit instruction such as “put the red cube in zone B” takes precedence
over the default mapping.

## Requirements

- Ubuntu 22.04 with ROS 2 Humble
- `colcon`, `rosdep`, MoveIt 2, Gazebo/Ignition integration, and the Universal
  Robots description and MoveIt configuration packages
- A running 9Router service or another compatible chat-completions endpoint
  for LLM-driven tasks

Install dependencies declared by the workspace with `rosdep`:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build and source the workspace:

```bash
cd ~/workspaces/ur_gz
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Run the letter-drawing demo

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_draw_letter draw_letter.launch.py \
  ur_type:=ur3e gazebo_gui:=true launch_rviz:=true
```

The node plans and executes a Cartesian path for the letter L. RViz shows the
robot and path markers; Gazebo shows the simulation. To run without the Gazebo
window, set `gazebo_gui:=false`.

## Run an LLM pick-and-place task

First provide the API key in the terminal without putting it in the command
history:

```bash
read -rsp "9Router API key: " NINEROUTER_API_KEY
printf '\n'
export NINEROUTER_API_KEY
```

For the hosted 9Router endpoint and the launch defaults:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' \
  student_id:=23020749
```

For a local OpenAI-compatible 9Router gateway, start that gateway separately,
then pass its endpoint and model explicitly:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Put the red cube in zone B.' \
  endpoint:=http://localhost:20128/v1/chat/completions \
  model:=ag/gemini-3.8-flash \
  student_id:=23020749
```

The launch starts Gazebo, MoveIt, and the gripper controller. The task node is
delayed until those services have time to start. To ask the planner to sort
the three mapped cubes by the student-ID mapping:

```bash
ros2 launch ur3_llm_control llm_robot.launch.py \
  command:='Arrange all objects according to my student ID.' \
  student_id:=23020749
```

To inspect planning and validation without moving the simulated robot, add
`execute:=false`.

## Configuration

- `src/ur3_llm_control/config/student_config.yaml`: student identity and
  coursework mapping.
- `src/ur3_llm_control/config/scene.yaml`: fixed camera/zone/table calibration; object poses are detected from RGB images.
- `src/ur3_llm_control/worlds/task_world.sdf`: Gazebo objects and task world.
- `src/ur3_draw_letter/urdf/ur.urdf.xacro`: UR3e mount and simulated gripper.
- `src/ur3_draw_letter/config/ur_controllers.yaml`: arm and gripper controllers.

API keys are intentionally not stored in this repository. Keep them in the
shell environment or a local secret manager and do not commit them.

## Troubleshooting

- **`NINEROUTER_API_KEY` is missing:** set/export it in the same terminal from
  which ROS is launched, or pass the launch argument `api_key:=...`.
- **Connection refused:** make sure the selected hosted/local LLM endpoint is
  running and reachable; for a local gateway, verify its port and model name.
- **Packages are not found:** source both `/opt/ros/humble/setup.bash` and
  `install/setup.bash` in the current terminal, and rebuild after source/config
  changes.
- **Grasp or placement fails:** inspect Gazebo contacts and the task-node log.
  Physical attachment acknowledgement depends on the Gazebo detachable-joint
  plugin and object contact state.

## Repository

GitHub: <https://github.com/linhnghu/ur3-llm-gripper-camera>
