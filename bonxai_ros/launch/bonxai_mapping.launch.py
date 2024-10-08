import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node

# this is the function launch  system will look for
def generate_launch_description():

    # Current Package Name
    package = "bonxai_ros"

    bonxai_params = os.path.join(
        get_package_share_directory(package),
        'params',
        'bonxai_params.yaml'
        )

    # Bonxai Server Node
    bonxai_node = Node(
        package=package,
        executable='bonxai_server_node',
        name='bonxai_server_node',
        emulate_tty=True,
        parameters=[bonxai_params],
        output="screen"
    )

    # RVIZ Configuration
    rviz_config_dir = os.path.join(get_package_share_directory(package), 'rviz', 'bonxai.rviz')

    rviz_node = Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            name='rviz_node',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_dir])

    # Launch Nodes
    return LaunchDescription(
        [
            bonxai_node,
            rviz_node,
        ]
    )
