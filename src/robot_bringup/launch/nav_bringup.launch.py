from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
    SetEnvironmentVariable
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """
    Full Navigation pipeline (requires a pre-built map):
      Gazebo  →  RSP  →  spawn robot  →  Nav2 (AMCL+planner+controller)  →  RViz

    Usage:
      ros2 launch robot_bringup nav_bringup.launch.py

    Before using this, make sure you have a map at:
      robot_nav2/maps/maze_map.yaml   (and maze_map.pgm)
    """

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_robot_gazebo = FindPackageShare('robot_gazebo')
    pkg_robot_description = FindPackageShare('robot_description')
    pkg_robot_nav2 = FindPackageShare('robot_nav2')
    pkg_robot_bringup = FindPackageShare('robot_bringup')

    world_path = PathJoinSubstitution([
        pkg_robot_gazebo, 'worlds', 'maze_1.world'
    ])
    xacro_file = PathJoinSubstitution([
        pkg_robot_description, 'urdf', 'robot.urdf.xacro'
    ])
    rviz_config = PathJoinSubstitution([
        pkg_robot_bringup, 'rviz', 'nav.rviz'
    ])
    gazebo_model_path = PathJoinSubstitution([
        pkg_robot_gazebo, 'models'
    ])

    robot_description = Command(['xacro ', xacro_file])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock'
        ),

        # 1. GAZEBO_MODEL_PATH
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[gazebo_model_path, ':']
        ),

        # 2. Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gazebo.launch.py'])
            ]),
            launch_arguments={
                'world': world_path,
                'verbose': 'false',
            }.items()
        ),

        # 3. Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_description,
            }]
        ),

        # 4. Spawn Robot (delayed 3 s)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='gazebo_ros',
                    executable='spawn_entity.py',
                    arguments=[
                        '-entity', 'slam_bot',
                        '-topic', 'robot_description',
                        '-x', '1.0',
                        '-y', '1.0',
                        '-z', '0.2',
                    ],
                    output='screen'
                )
            ]
        ),

        # 5. Nav2 Full Stack (delayed 5 s)
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            pkg_robot_nav2, 'launch', 'nav2_bringup.launch.py'
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                    }.items()
                )
            ]
        ),

        # 6. RViz (delayed 5 s)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='rviz2',
                    executable='rviz2',
                    name='rviz2',
                    output='screen',
                    arguments=['-d', rviz_config],
                    parameters=[{'use_sim_time': use_sim_time}]
                )
            ]
        ),
    ])
