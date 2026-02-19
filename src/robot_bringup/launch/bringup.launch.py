from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # ---- WORLD PATH ----
    world_path = PathJoinSubstitution([
        FindPackageShare('robot_gazebo'),
        'worlds',
        'maze_1.world'
    ])

    # ---- ROBOT DESCRIPTION FROM XACRO ----
    robot_description = Command([
        'xacro ',
        PathJoinSubstitution([
            FindPackageShare('robot_description'),
            'urdf',
            'robot.urdf.xacro'
        ])
    ])

    # ---- ROBOT STATE PUBLISHER ----
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True
        }],
        output='screen'
    )

    # ---- GAZEBO ----
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gazebo.launch.py'
            ])
        ),
        launch_arguments={
            'world': world_path
        }.items()
    )

    # ---- SPAWN ROBOT (Delayed) ----
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'slam_bot',
            '-topic', 'robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'   # spawn slightly above ground to avoid collision bounce
        ],
        output='screen'
    )

    # ---- RVIZ ----
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        parameters=[{'use_sim_time': True}],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        rsp_node,
        TimerAction(
            period=4.0,   # wait for gazebo physics to load
            actions=[spawn_robot]
        ),
        rviz
    ])
