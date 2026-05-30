import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
from launch.actions import ExecuteProcess

def generate_launch_description():
    
    pkg_dir = get_package_share_directory('robot_description')

    xacro_file = os.path.join(
        pkg_dir,
        'urdf',
        'modelone.urdf.xacro'
    )

    # Process xacro properly
    robot_desc = xacro.process_file(
        xacro_file
    ).toxml()

    rsp = Node(
        name='robot_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_desc
        }],
        output='screen'
    )

    jsp = Node(
        name='joint_state_publisher',
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # Launch Gazebo/Ignition
    # gazebo = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=['empty.sdf'],
    #     output='screen'
    # )

    # # Spawn robot
    # spawn = Node(
    #     package='ros_gz_sim',
    #     executable='create',
    #     arguments=[
    #         '--topic', '/robot_description',
    #         '--name', 'modelone'
    #     ],
    #     output='screen'
    # )
    ign_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            )
        ]),
        # Pass default arguments to Gazebo; here we load an empty world
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', '/robot_description',
            '-name', 'modelone'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        jsp,
        ign_gazebo_launch,
        spawn_robot
    ])