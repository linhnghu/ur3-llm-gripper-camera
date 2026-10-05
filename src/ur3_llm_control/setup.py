from setuptools import find_packages, setup

package_name = "ur3_llm_control"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/ur3_llm_control"]),
        ("share/ur3_llm_control", ["package.xml"]),
        ("share/ur3_llm_control/launch", ["launch/llm_robot.launch.py"]),
        ("share/ur3_llm_control/config", ["config/scene.yaml", "config/student_config.yaml"]),
        ("share/ur3_llm_control/worlds", ["worlds/task_world.sdf"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "llm_task_node=ur3_llm_control.llm_task_node:main",
        "llm_command=ur3_llm_control.command_cli:main",
    ]},
)
