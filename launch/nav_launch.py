from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("nav2_bringup"),
                    "launch",
                    "bringup_launch.py"
                ])
            ]),
            launch_arguments={
                "use_sim_time": "true",
                "map": PathJoinSubstitution([
                    FindPackageShare("prm"), "maps", "tb3_sandbox.yaml"
                ]),
                "autostart": "true"
            }.items()
        )
    ])

from launch_ros.actions import Node

amcl_node = Node(
    package="nav2_amcl",
    executable="amcl",
    name="amcl",
    output="screen",
    parameters=[
        {"use_sim_time": True},
        PathJoinSubstitution([
            FindPackageShare("prm"), "config", "nav2_params.yaml"
        ])
    ]
)

def generate_launch_description():
    return LaunchDescription([
        # seu IncludeLaunchDescription do bringup_launch.py aqui
        amcl_node
    ])

