import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch_ros.actions import Node, SetParameter
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution

import xacro

def generate_launch_description():
    ld = LaunchDescription()

    # Specify the name of the package and path to xacro file within the package
    pkg_name = 'real_leo'

    # Set ignition resource path (so it can find your world files)
    ign_resource_path = SetEnvironmentVariable(name='IGN_GAZEBO_RESOURCE_PATH',
    value=[os.path.join(get_package_share_directory(pkg_name),'worlds')])

    # Include the Gazebo launch file
    launch_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('ros_gz_sim'), '/launch', '/gz_sim.launch.py']),
    launch_arguments={
    'gz_args' : '-r empty.sdf'
    }.items(),
    )

    robot_desc = xacro.process_file(
        os.path.join(
            get_package_share_directory(pkg_name),
            "urdf",
            "leo_sim.urdf.xacro",
        ),
    ).toxml()

    rplidar = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory('rplidar_ros'), '/launch', '/rplidar_a2m12_launch.py']),
    launch_arguments={}.items(),
    )

    # Launch robot state publisher node
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output='screen',
        parameters=[
            {"robot_description": robot_desc},
        ],
    )

    ekf_localization = Node(
       package="robot_localization",
       executable="ekf_node",
       name="ekf_node",
       output = 'screen',
       parameters=[os.path.join(get_package_share_directory(pkg_name), 'config', 'ekf_node.yaml')],
    )

    # Rviz node
    node_rviz = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/rviz_leo.launch.py']),
    launch_arguments={}.items(),
    )

    navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/navigation.launch.py']),
    launch_arguments={}.items(),
    )

    joy_stick = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([get_package_share_directory(pkg_name), '/launch', '/teleop_joy.launch.py']),
    launch_arguments={}.items(),
    )

    # Spawn a robot inside a simulation
    leo_rover = Node(
        # namespace="leo_rover",
        package="ros_gz_sim",
        executable="create",
        # name="ros_gz_sim_create",
        # output="both",
        output='screen',
        arguments=[
            "-topic",
            "/robot_description",
            # "-name",
            # "leo_rover",
            "-z",
            "0.5",
        ],
    )


    # Add actions to LaunchDescription
    ld.add_action(SetParameter(name='use_sim_time', value=False))
    ld.add_action(launch_gazebo)
    ld.add_action(rplidar)
    ld.add_action(robot_state_publisher)
    ld.add_action(leo_rover)
    ld.add_action(ekf_localization)
    ld.add_action(node_rviz)
    ld.add_action(navigation)
    ld.add_action(joy_stick)

    
    return ld
