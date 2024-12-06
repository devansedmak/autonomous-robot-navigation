from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    
    ld = LaunchDescription()

    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='False')
    ld.add_action(use_sim_time)
    namespace = DeclareLaunchArgument('namespace', default_value='myrobot') 
    ld.add_action(namespace)
    default_config = os.path.join(get_package_share_directory('core_tue4tm00_assignment2'), 'config', 'search_based_path_planner.yaml') 
    config = DeclareLaunchArgument('config', default_value=default_config)
    ld.add_action(config)
    pose = DeclareLaunchArgument('pose', default_value='pose')
    ld.add_action(pose)
    goal = DeclareLaunchArgument('goal', default_value='goal')
    ld.add_action(goal)
    scan = DeclareLaunchArgument('scan', default_value='scan')
    ld.add_action(scan)
    map = DeclareLaunchArgument('map', default_value='map')
    ld.add_action(map)
    costmap = DeclareLaunchArgument('costmap', default_value='costmap')
    ld.add_action(costmap)
    path = DeclareLaunchArgument('path', default_value='path')
    ld.add_action(path)


    path_planner_node = Node(
        package='core_tue4tm00_assignment2',
        executable='search_based_path_planner.py',
        name='path_planner',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            },
        ],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
            ('pose', LaunchConfiguration('pose')),
            ('goal', LaunchConfiguration('goal')),
            ('scan', LaunchConfiguration('scan')),
            ('map', LaunchConfiguration('map')),
            ('costmap', LaunchConfiguration('costmap')),
            ('path', LaunchConfiguration('path')),
        ],
        output='screen',
        emulate_tty=True,
    )
    ld.add_action(path_planner_node)

    return ld