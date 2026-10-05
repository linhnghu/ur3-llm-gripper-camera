import os
import shlex
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("ur3_llm_control")
    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_simulation_gz"), "/launch/ur_sim_control.launch.py"]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "runtime_config_package": "ur3_draw_letter",
            "controllers_file": "ur_controllers.yaml",
            "description_package": "ur3_draw_letter",
            "description_file": "ur3_table_camera.urdf.xacro",
            "launch_rviz": "false",
            "gazebo_gui": LaunchConfiguration("gazebo_gui"),
            "world_file": PathJoinSubstitution([share, "worlds", "task_world.sdf"]),
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare("ur_moveit_config"), "/launch/ur_moveit.launch.py"]),
        launch_arguments={
            "ur_type": LaunchConfiguration("ur_type"),
            "safety_limits": "true",
            # Use the same custom URDF (including gripper links) as simulation.
            "description_package": "ur3_draw_letter",
            "description_file": "ur3_table_camera.urdf.xacro",
            "moveit_config_package": "ur3_draw_letter",
            "moveit_config_file": "ur.srdf.xacro",
            "use_sim_time": "true",
            "launch_rviz": "false",
            "launch_servo": "false",
        }.items(),
    )
    robot_description = ParameterValue(Command([
        FindExecutable(name="xacro"), " ", FindPackageShare("ur3_draw_letter"),
        "/urdf/ur3_table_camera.urdf.xacro name:=ur ur_type:=",
        LaunchConfiguration("ur_type"), " safety_limits:=true"]), value_type=str)
    # VS Code Snap can export its own libc/Qt paths. Give RViz system libraries
    # while preserving the user's ROS domain and workspace package index.
    rviz_environment = {
        "PATH": "/opt/ros/humble/bin:/usr/bin:/bin",
        "HOME": os.environ.get("HOME", "/tmp"),
        "DISPLAY": os.environ.get("DISPLAY", ":0"),
        "XDG_RUNTIME_DIR": os.environ.get("XDG_RUNTIME_DIR", f"/run/user/{os.getuid()}"),
        "QT_QPA_PLATFORM": "xcb",
        "LANG": "C.UTF-8",
        "LD_LIBRARY_PATH": "/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:"
                           "/opt/ros/humble/opt/rviz_ogre_vendor/lib:/usr/lib/x86_64-linux-gnu",
        "AMENT_PREFIX_PATH": os.environ.get("AMENT_PREFIX_PATH", "/opt/ros/humble"),
        "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID", "0"),
        "ROS_DISTRO": "humble", "ROS_VERSION": "2",
        "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
    }
    for variable in ("XAUTHORITY", "ROS_LOG_DIR"):
        if variable in os.environ:
            rviz_environment[variable] = os.environ[variable]
    rviz = Node(
        package="rviz2", executable="rviz2", output="screen", name="rviz2_moveit",
        condition=IfCondition(LaunchConfiguration("launch_rviz")),
        prefix=["env -i "] + [shlex.quote(f"{key}={value}") + " "
                              for key, value in rviz_environment.items()],
        arguments=["-d", PathJoinSubstitution([FindPackageShare("ur3_draw_letter"), "rviz", "view_robot.rviz"])],
        parameters=[{
            "robot_description": robot_description,
            "robot_description_semantic": ParameterValue(Command([
                FindExecutable(name="xacro"), " ", FindPackageShare("ur3_draw_letter"),
                "/srdf/ur.srdf.xacro name:=ur"]), value_type=str),
            "use_sim_time": True,
        }, PathJoinSubstitution([FindPackageShare("ur3_draw_letter"), "config", "kinematics.yaml"])],
    )
    task_node = Node(
        package="ur3_llm_control", executable="llm_task_node",
        name="llm_task_planner", output="screen",
        parameters=[{
            "robot_description": robot_description,
            "command": LaunchConfiguration("command"),
            "api_key": LaunchConfiguration("api_key"),
            "endpoint": LaunchConfiguration("endpoint"),
            "model": LaunchConfiguration("model"),
            "execute": ParameterValue(LaunchConfiguration("execute"), value_type=bool),
            "demo_plan": ParameterValue(LaunchConfiguration("demo_plan"), value_type=bool),
            "continuous": ParameterValue(LaunchConfiguration("continuous"), value_type=bool),
            "record_path": LaunchConfiguration("record_path"),
            "evidence_path": LaunchConfiguration("evidence_path"),
            "use_sim_time": True,
            "student_id": ParameterValue(LaunchConfiguration("student_id"), value_type=str),
        }],
    )
    gripper_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=[
            "gripper_controller", "-c", "/controller_manager",
            "--controller-manager-timeout", "60", "--service-call-timeout", "60",
            "--switch-timeout", "60",
        ],
        output="screen",
    )
    bridge_arguments = ["/table_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image"]
    for obj in ("red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"):
        prefix = f"/ur3_llm/grasp/{obj}"
        bridge_arguments += [prefix + "/state@std_msgs/msg/String[ignition.msgs.StringMsg",
                             prefix + "/attach@std_msgs/msg/Empty]ignition.msgs.Empty",
                             prefix + "/detach@std_msgs/msg/Empty]ignition.msgs.Empty"]
    camera_bridge = Node(package="ros_gz_bridge", executable="parameter_bridge",
                         arguments=bridge_arguments, output="screen")
    return LaunchDescription([
        SetEnvironmentVariable("IGN_GAZEBO_SYSTEM_PLUGIN_PATH", PathJoinSubstitution([
            FindPackageShare("ur3_grasp_plugin"), "..", "..", "lib"])),
        DeclareLaunchArgument("ur_type", default_value="ur3e"),
        DeclareLaunchArgument("gz_partition", default_value=f"ur3_llm_{os.getpid()}"),
        DeclareLaunchArgument("gazebo_gui", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument("command", default_value=""),
        DeclareLaunchArgument("api_key", default_value=""),
        DeclareLaunchArgument("endpoint", default_value="https://9router.com/v1/chat/completions"),
        DeclareLaunchArgument("model", default_value="gpt-4o-mini"),
        DeclareLaunchArgument("execute", default_value="true"),
        DeclareLaunchArgument("student_id", default_value="23020749"),
        DeclareLaunchArgument("demo_plan", default_value="false"),
        DeclareLaunchArgument("continuous", default_value="false"),
        DeclareLaunchArgument("record_path", default_value=""),
        DeclareLaunchArgument("evidence_path", default_value=""),
        # Isolate the transport world from earlier instances during restart.
        SetEnvironmentVariable("IGN_PARTITION", LaunchConfiguration("gz_partition")),
        GroupAction(scoped=True, actions=[simulation]),
        GroupAction(scoped=True, actions=[moveit]),
        camera_bridge,
        rviz,
        TimerAction(period=18.0, actions=[gripper_spawner]),
        TimerAction(period=40.0, actions=[task_node]),
    ])
