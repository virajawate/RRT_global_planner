import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro

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
    gazebo = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['empty.sdf'],
        output='screen'
    )

    # Spawn robot
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '--topic', '/robot_description',
            '--name', 'modelone'
        ],
        output='screen'
    )

    return LaunchDescription([
        rsp,
        jsp,
        # gazebo,
        spawn
    ])