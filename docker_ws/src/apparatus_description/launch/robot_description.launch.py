import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Find package
    pkg_share = FindPackageShare(package='apparatus_description').find('apparatus_description')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    simulation = LaunchConfiguration('simulation', default='false')
    
    # Path to URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false'))
    ld.add_action(DeclareLaunchArgument('simulation', default_value='false'))
    
    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': urdf_file},
        ],
    )
    
    # Node: joint_state_publisher
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    
    return ld
