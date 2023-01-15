#!/usr/bin/env python3
# Authors: Deebul Nair

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchContext()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    urdf_file_name = '4_wheel_config.urdf.xacro'

    print('urdf_file_name : {}'.format(urdf_file_name))


    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("robile_description"),
                    "robots",
                    urdf_file_name
                ]
            ),
        ]
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': ParameterValue(robot_description_content, value_type=str)
            }],
    )

    spawn_robot_gazebo_cmd = Node(package=    'gazebo_ros',
                                  executable= 'spawn_entity.py',
                                  arguments=  ['-topic', 'robot_description',
                                               '-entity', 'robile',
                                              ],
                                  output=     'screen')

    nodes = [
        gzserver_cmd,
        gzclient_cmd,
        robot_state_pub_node,
        spawn_robot_gazebo_cmd
    ]

    return LaunchDescription(nodes)

    #urdf_path = os.path.join(
    #    get_package_share_directory('robile_description'),
    #    'robots',
    #    urdf_file_name)

    #with open(urdf_path, 'r') as infp:
    #    robot_desc = infp.read()


    #return LaunchDescription([
    #    DeclareLaunchArgument(
    #        'use_sim_time',
    #        default_value='false',
    #        description='Use simulation (Gazebo) clock if true'),

    #    Node(
    #        package='robot_state_publisher',
    #        executable='robot_state_publisher',
    #        name='robot_state_publisher',
    #        output='screen',
    #        parameters=[{
    #            'use_sim_time': use_sim_time,
    #            'robot_description': robot_desc
    #        }],
    #    ),
    #])
