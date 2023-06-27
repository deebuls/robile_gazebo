import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

NUM_ROBOTS = 2


def gen_robot_list(number_of_robots):
    
    robots = []
    
    for i in range(number_of_robots):
        robot_name = "robile_"+ str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01})

    return robots 



def generate_launch_description():

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_pathfinder = get_package_share_directory('robile_gazebo')
    description_path = os.path.join(get_package_share_directory('robile_description'))
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Get the URDF xacro file path
    xacro_file = xacro.process_file(os.path.join(description_path, 'gazebo/', 'gazebo_robile.xacro'),
                                    mappings={'platform_config':'4_wheel_config', 'movable_joints':'False'})

    urdf_path = os.path.join(description_path, 'gazebo/', 'full_robile_gazebo.urdf')

    urdf_file = open(urdf_path, "w")
    urdf_file.write(xacro_file.toxml())
    urdf_file.close()
    

    # Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py'),
        )
    )

    # Spawn multiple pathfinder's in a row
    #spawn_robots = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_pathfinder, 'launch', 'n_robiles_spawner.launch.py'),
    #    )
    #)     

    # Get the SLAM parameters yaml file path
    #slam_params_path = os.path.join(pkg_path, 'config/', 'slam_params_online_async.yaml')
    slam_params_path = ''

    robots = gen_robot_list(NUM_ROBOTS) # list of robots with names and poses:
    spawn_robots_cmds = [] # list of commands to spawn robots

    for robot in robots:
        robot_name = robot['name']

        group_cmds = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(pkg_pathfinder , 'launch', 'generic_gazebo_4_wheel.launch.py')),
                launch_arguments={
                                  'use_sim_time': use_sim_time,
                                  'robot_name': robot_name,
                                  'robot_namespace': robot_name,
                                  # tf_ remapping depend if you want separate /tf for each robot
                                  'tf_remapping': '/'+robot_name+'/tf',
                                  #'tf_remapping': '/tf',
                                  'static_tf_remap': '/'+robot_name+'/tf_static',
                                  #'static_tf_remap': '/tf_static',
                                  'scan_topic': '/'+robot_name+'/scan',
                                  'map_topic': '/'+robot_name+'/map',
                                  'odom_topic': '/'+robot_name+'/odom',
                                  'slam_params': slam_params_path,
                                  'frame_prefix': robot_name+'/',
                                  'urdf': open(urdf_path).read(),
                                  'urdf_path': urdf_path,
                                  'x': TextSubstitution(text=str(robot['x_pose'])),
                                  'y': TextSubstitution(text=str(robot['y_pose'])),
                                  'z': TextSubstitution(text=str(robot['z_pose']))
                                  }.items()), ])
        
        spawn_robots_cmds.append(group_cmds)

    ld =  LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value=[os.path.join(pkg_pathfinder, 'worlds', 'cart_basement.world') ],
          description='SDF world file'),
        DeclareLaunchArgument(
          'use_sim_time',
          default_value='true',
          description='Use simulation/Gazebo clock'),
        gazebo,
    ])

    # Add each cmd as a new "launch action" to the launch description
    for spawn_robot_cmd in spawn_robots_cmds:
        ld.add_action(spawn_robot_cmd)

    return ld
