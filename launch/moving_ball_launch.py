import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def generate_launch_description() -> LaunchDescription:
    pkg_name = 'f1tenth_gazebo'
    xacro_filename = 'f1tenth.urdf.xacro'
    
    # Define Launch Configurations
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='-1.1')
    z_pose = LaunchConfiguration('z_pose', default='0.1')

    # Declare Launch Arguments
    declare_x_pose_cmd = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Specify x-coordinate of the car')
    declare_y_pose_cmd = DeclareLaunchArgument(
        'y_pose', default_value='-1.1', description='Specify y-coordinate of the car')
    declare_z_pose_cmd = DeclareLaunchArgument(
        'z_pose', default_value='0.1', description='Specify z-coordinate of the car')

    # Process the xacro file for the f1tenth car
    xacro_file = os.path.join(get_package_share_directory(pkg_name), 'urdf', xacro_filename)
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()} # type: ignore

    # Start the robot_state_publisher node
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Include the Gazebo launch file
    world_file = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'moving_ball.world')

    gazebo_pkg = get_package_share_directory('gazebo_ros')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(gazebo_pkg, 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )

    reset_car_node = Node(
        package='f1tenth_gazebo',
        executable='reset_car',
        output='screen'
    )

    ackermann_to_drive_node = Node(
        package='f1tenth_gazebo',
        executable = 'convert_drive',
        output='screen',
    )

    # Spawn the f1tenth car using the parameterized coordinates
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'f1tenth_car',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose
        ],
        output='screen'
    )

    # 3. Add the declare commands to the LaunchDescription
    return LaunchDescription([
        declare_x_pose_cmd,
        declare_y_pose_cmd,
        declare_z_pose_cmd,
        ackermann_to_drive_node,
        node_robot_state_publisher,
        gazebo,
        spawn_entity
    ])