import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch_ros.descriptions import ParameterValue
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


import launch.actions
import launch_ros.actions
import launch.substitutions
from launch_ros.descriptions import ParameterValue


def generate_launch_description():

    pkg_path = os.path.join(get_package_share_directory('robile_gazebo'))
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_pathfinder = get_package_share_directory('robile_gazebo')
    description_path = os.path.join(get_package_share_directory('robile_description'))
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )


    # Get the URDF xacro file path
    xacro_file = xacro.process_file(os.path.join(description_path, 'gazebo/', 'gazebo_robile.xacro'),
                                    mappings={'platform_config':'4_wheel_config', 'movable_joints':'False'})

    urdf_path = os.path.join(description_path, 'gazebo/', 'full_robile_gazebo.urdf')

    urdf_file = open(urdf_path, "w")
    urdf_file.write(xacro_file.toxml())
    urdf_file.close()
    
    # Get the SLAM parameters yaml file path
    #slam_params_path = os.path.join(pkg_path, 'config/', 'slam_params_online_async.yaml')
    slam_params_path = ''

    robile = Node(
            package='robile_gazebo',
            executable='generic_spawn.py',
            output='screen',
            arguments=[ '--urdf_path', urdf_path, 
                #'--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                #'--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                #'-x', launch.substitutions.LaunchConfiguration('x'),
                #'-y', launch.substitutions.LaunchConfiguration('y'),
                #'-z', launch.substitutions.LaunchConfiguration('z')
                ])

    robile_state_pub = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            #namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            #removing the remapping get the combineed tf published 
            #remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
            #            ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap'))],
            parameters=[{
                #'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                'robot_description': ParameterValue(open(urdf_path).read(), value_type=str),
                #'frame_prefix': launch.substitutions.LaunchConfiguration('frame_prefix'),
                'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'), }])
        
    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_pathfinder, 'worlds', 'cart_basement.world') ],
          description='SDF world file'),
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'),
        gazebo,
        robile,
        robile_state_pub
    ])
