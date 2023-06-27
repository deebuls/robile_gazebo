from launch import LaunchDescription

import launch.actions
import launch_ros.actions
import launch.substitutions
import launch.launch_description_sources
from launch_ros.descriptions import ParameterValue
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    def launch_setup(context, *args, **kwargs):

        robot_name = launch.substitutions.LaunchConfiguration('robot_name').perform(context)

        return [
            launch_ros.actions.Node(
                package='robile_gazebo',
                executable='generic_spawn.py',
                output='screen',
                arguments=[
                    '--urdf_path', launch.substitutions.LaunchConfiguration('urdf_path'),
                    '--robot_name', launch.substitutions.LaunchConfiguration('robot_name'),
                    '--robot_namespace', launch.substitutions.LaunchConfiguration('robot_namespace'),
                    '-x', launch.substitutions.LaunchConfiguration('x'),
                    '-y', launch.substitutions.LaunchConfiguration('y'),
                    '-z', launch.substitutions.LaunchConfiguration('z')]),

            launch_ros.actions.Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                output='screen',
                namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
                #removing the remapping get the combineed tf published 
                remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                            ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap'))],
                parameters=[{
                    #'robot_description': launch.substitutions.LaunchConfiguration('urdf'),
                    'robot_description': ParameterValue(launch.substitutions.LaunchConfiguration('urdf'), value_type=str),
                    'frame_prefix': launch.substitutions.LaunchConfiguration('frame_prefix'),
                    'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time'), }]),
           
            launch_ros.actions.Node(
                package='rviz2',
                namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
                executable='rviz2',
                #name='rviz2',
                arguments=['-d', PathJoinSubstitution(
                                     [FindPackageShare("robile_gazebo"), "config", "rviz", robot_name+".rviz"]
                              )],
                remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
                            ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap')),
                            ('/clicked_point','/'+robot_name+'/clicked_point'),
                            ('/goal_pose','/'+robot_name+'/goal_pose'),
                            ('/initialpose','/'+robot_name+'/initialpose'),
                            ('/parameter_events','/'+robot_name+'/parameter_events'),
                           ],
                parameters=[{'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}]),
            #launch_ros.actions.Node(
            #    package='slam_toolbox',
            #    executable='async_slam_toolbox_node',
            #    name='slam_toolbox',
            #    output='screen',
            #    namespace=launch.substitutions.LaunchConfiguration('robot_namespace'),
            #    remappings=[('/tf',launch.substitutions.LaunchConfiguration('tf_remapping')),
            #                ('/tf_static',launch.substitutions.LaunchConfiguration('static_tf_remap')),
            #                ('/scan', launch.substitutions.LaunchConfiguration('scan_topic')),
            #                ('/map', launch.substitutions.LaunchConfiguration('map_topic')),
            #                ('/odom',launch.substitutions.LaunchConfiguration('odom_topic'))],
            #    parameters=[launch.substitutions.LaunchConfiguration('slam_params'),
            #                {'use_sim_time': launch.substitutions.LaunchConfiguration('use_sim_time')}])

        ]

    return LaunchDescription([

        launch.actions.OpaqueFunction(function=launch_setup, args=[launch.substitutions.LaunchConfiguration('robot_name')]),


    ])
