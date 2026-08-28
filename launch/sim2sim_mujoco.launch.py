import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    pkg_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_policy = os.path.join(pkg_dir, "models", "policy.pt")
    sim_script = os.path.join(pkg_dir, "sim2sim", "sim2sim_mujoco.py")

    policy_path_arg = DeclareLaunchArgument(
        "policy_path",
        default_value=default_policy,
        description="Path to TorchScript policy.pt",
    )
    terrain_arg = DeclareLaunchArgument(
        "terrain",
        default_value="flat",
        description="Terrain type for MuJoCo simulation: 'flat', 'rough', 'stairs', 'obstacles'",
    )
    task_arg = DeclareLaunchArgument(
        "task",
        default_value="rough",
        description="Task model type: 'rough' or 'flat'",
    )

    policy_path = LaunchConfiguration("policy_path")
    terrain = LaunchConfiguration("terrain")
    task = LaunchConfiguration("task")

    mujoco_process = ExecuteProcess(
        cmd=[
            "python3",
            sim_script,
            "--policy", policy_path,
            "--terrain", terrain,
            "--task", task,
        ],
        output="screen",
    )

    return LaunchDescription([
        policy_path_arg,
        terrain_arg,
        task_arg,
        mujoco_process,
    ])
