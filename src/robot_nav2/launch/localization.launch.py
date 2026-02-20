from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Map file path
    map_file = PathJoinSubstitution([
        FindPackageShare('robot_nav2'),
        'maps',
        'maze_map.yaml'
    ])

    # Nav2 parameter file
    nav2_params = PathJoinSubstitution([
        FindPackageShare('robot_nav2'),
        'config',
        'nav2_params.yaml'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # ---- Map Server ----------------------------------------
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[
                nav2_params,
                {
                    'use_sim_time': use_sim_time,
                    'yaml_filename': map_file
                }
            ]
        ),

        # ---- AMCL Localization ---------------------------------
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_params, {'use_sim_time': use_sim_time}]
        ),

        # ---- Lifecycle Manager ---------------------------------
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])