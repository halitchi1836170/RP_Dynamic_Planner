"""Avvia il nodo di navigazione con il file di parametri di default."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = Path(get_package_share_directory("marrtino_nav")) / "config" / "nav_params.yaml"
    return LaunchDescription([
        Node(
            package="marrtino_nav",
            executable="nav_node",
            name="marrtino_nav",
            output="screen",
            parameters=[str(params)],
        )
    ])
