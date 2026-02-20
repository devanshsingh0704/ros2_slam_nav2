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

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    pkg_gazebo_ros = FindPackageShare('gazebo_ros')
    pkg_robot_gazebo = FindPackageShare('robot_gazebo')
    pkg_robot_description = FindPackageShare('robot_description')

    world_path = PathJoinSubstitution([
        pkg_robot_gazebo, 'worlds', 'maze_1.world'
    ])

    xacro_file = PathJoinSubstitution([
        pkg_robot_description, 'urdf', 'robot.urdf.xacro'
    ])

    robot_description = Command(['xacro ', xacro_file])

    gazebo_model_path = PathJoinSubstitution([
        pkg_robot_gazebo, 'models'
    ])

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation clock if true'
        ),

        # ---- Set GAZEBO_MODEL_PATH so model://maze_1 resolves --
        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[gazebo_model_path, ':']
        ),

        # ---- Gazebo (via ros2 launch gazebo_ros) ---------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    pkg_gazebo_ros, 'launch', 'gazebo.launch.py'
                ])
            ]),
            launch_arguments={
                'world': world_path,
                'verbose': 'false',
            }.items()
        ),

        # ---- Robot State Publisher -----------------------------
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

        # ---- Spawn Robot (delayed 3 s) -------------------------
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
    ])
