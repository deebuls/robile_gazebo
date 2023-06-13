#!/usr/bin/env python3
# Authors: Deebul Nair

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue


def generate_launch_description():
    lc = LaunchContext()
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')


    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    domain_id = LaunchConfiguration('domain_id')
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_rviz = LaunchConfiguration('use_rviz')
    x_pose = LaunchConfiguration('x_pose', default='-0.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.0')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    
    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ')

    declare_domain_id_argument = DeclareLaunchArgument(
		'domain_id',
        default_value='',
        description='Domain ID in which the node has to be started'
	)

    declare_robot_pose_x_argument = DeclareLaunchArgument(
		'x_pose',
        default_value='0.0',
        description='robot pose x '
	)
    declare_robot_pose_y_argument = DeclareLaunchArgument(
		'y_pose',
        default_value='0.0',
        description='robot pose y'
	)




    world = os.path.join(
        get_package_share_directory('robile_gazebo'),
        'worlds',
        'cart_basement.world'
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
                    "gazebo",
                    "gazebo_robile.xacro"
                ]
            ),
            " ",
            "platform_config:=4_wheel_config",
            " ",
            "movable_joints:=False",
        ]
    )

    robot_state_pub_cmd = Node(
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
                                               '-x', x_pose,
                                               '-y', y_pose,
                                               '-Y', '0.78',
                                               '-timeout', '180'
                                              ],
                                  output=     'screen')

    rviz_cmd = Node(package='rviz2',
                    namespace='',
                    executable='rviz2',
                    name='rviz2',
                    parameters=[{'use_sim_time': use_sim_time}],
    )

    zenoh_dds_brigde_process = ExecuteProcess(
        cmd=['zenoh-bridge-dds', '-d', domain_id] #make robile name configurable
    )
    
    print ("Use rviz ", use_rviz)
    nodes = [
        #SetEnvironmentVariable('ROS_DOMAIN_ID', domain_id),
        rviz_cmd,
        gzserver_cmd,
        gzclient_cmd,
        robot_state_pub_cmd,
        spawn_robot_gazebo_cmd,
        #zenoh_dds_brigde_process,
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
