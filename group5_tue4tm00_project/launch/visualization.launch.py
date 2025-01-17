from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    # Declare arguments
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    ld.add_action(use_sim_time)
    namespace = DeclareLaunchArgument('namespace', default_value='myrobot')
    ld.add_action(namespace)
    default_config = os.path.join(get_package_share_directory('group5_tue4tm00_project'), 'config', 'visualization.yaml') 
    config = DeclareLaunchArgument('config', default_value=default_config)
    ld.add_action(config)
    pose = DeclareLaunchArgument('pose', default_value='pose')
    ld.add_action(pose)
    goal = DeclareLaunchArgument('goal', default_value='goal')
    ld.add_action(goal)
    scan = DeclareLaunchArgument('scan', default_value='scan')
    ld.add_action(scan)
    path = DeclareLaunchArgument('path', default_value='path')
    ld.add_action(path)
    convex_interior = DeclareLaunchArgument('convex_interior', default_value='convex_interior')
    ld.add_action(convex_interior)
    path_goal = DeclareLaunchArgument('path_goal', default_value='path_goal')
    ld.add_action(path_goal)
    pose_in = DeclareLaunchArgument('pose_in', default_value='pose_in')
    ld.add_action(pose_in)

    # Visualization Node
    visualization_node = Node(
        package='group5_tue4tm00_project',
        executable='visualization.py',
        name='visualization',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time')
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('pose', LaunchConfiguration('pose')),
            ('goal', LaunchConfiguration('goal')),
            ('scan', LaunchConfiguration('scan')),
            ('path', LaunchConfiguration('path')),
            ('convex_interior', LaunchConfiguration('convex_interior')),
            ('path_goal', LaunchConfiguration('path_goal')),
            ('pose_in', LaunchConfiguration('pose_in'))
        ],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(visualization_node)

    return ld
