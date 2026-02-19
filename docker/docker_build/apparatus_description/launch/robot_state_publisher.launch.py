import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition


def generate_launch_description():
    # Find package
    pkg_share = FindPackageShare(package='apparatus_description').find('apparatus_description')
    
    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    simulation = LaunchConfiguration('simulation', default='false')
    gui = LaunchConfiguration('gui', default='true')
    
    # Path to URDF file
    urdf_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    
    # Generate URDF from Xacro
    robot_description_command = [
        'xacro ',
        urdf_file,
        ' use_sim_time:=',
        use_sim_time,
        ' simulation:=',
        simulation,
    ]
    
    # Launch description
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='false',
                                        description='Use simulation (Gazebo) clock if true'))
    ld.add_action(DeclareLaunchArgument('simulation', default_value='false',
                                        description='Run simulation if true'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true',
                                        description='Show joint_state_publisher GUI'))
    
    # Node: robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'robot_description': ParameterValue(Command(robot_description_command), value_type=str)},
        ],
    )
    
    # Node: joint_state_publisher without GUI (fallback)
    joint_state_publisher_no_gui_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )
    
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_no_gui_node)
    
    return ld
