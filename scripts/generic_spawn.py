#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""Script used to spawn a robot at a generic position in a Gazebo world.
    Requires Gazebo running during execution.
"""
import os
import rclpy
import argparse
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


def main():
    # Get input arguments from user
    parser = argparse.ArgumentParser(description='Spawn a robot into the Gazebo world')
    parser.add_argument('-urdf', '--urdf_path', type=str, default='',
                        help='Name of the robot to spawn')
    parser.add_argument('-n', '--robot_name', type=str, default='',
                        help='Name of the robot to spawn')
    parser.add_argument('-ns', '--robot_namespace', type=str, default='',
                        help='ROS namespace to apply to the tf and plugins')
    parser.add_argument('-namespace', '--namespace', type=bool, default=True,
                        help='Whether to enable namespacing')
    parser.add_argument('-x', type=float, default=0,
                        help='the x component of the initial position [meters]')
    parser.add_argument('-y', type=float, default=0,
                        help='the y component of the initial position [meters]')
    parser.add_argument('-z', type=float, default=0,
                        help='the z component of the initial position [meters]')

    args, unknown = parser.parse_known_args()

    # Start node
    rclpy.init()
    node_name = args.robot_name + '_spawner'
    node = rclpy.create_node(node_name)

    node.get_logger().info(
        'Creating Service client to connect to `/spawn_entity`')
    client = node.create_client(SpawnEntity, '/spawn_entity')

    node.get_logger().info('Connecting to `/spawn_entity` service...')
    if not client.service_is_ready():
        client.wait_for_service()
        node.get_logger().info('...connected!')

    urdf_file_path = args.urdf_path
    tree = ET.parse(urdf_file_path)
    root = tree.getroot()

    if args.robot_name is not '':
        planar_move_plugin = None 
        for plugin in root.iter('plugin'):
            if 'gazebo_ros_planar_move' in plugin.attrib.values():
                planar_move_plugin = plugin
                break

        # We change the namespace to the robots corresponding one
        tag_ros_params = planar_move_plugin.find('ros')
        tag_ns = ET.SubElement(tag_ros_params, 'namespace')
        tag_ns.text = '/' + args.robot_name
        ros_tf_remap = ET.SubElement(tag_ros_params, 'remapping')   #Informs gazebo to puslish tf in different topic
        ros_tf_remap.text = '/tf:=/' + args.robot_name + '/tf'
        ros_tf_static_remap = ET.SubElement(tag_ros_params, 'remapping')   #Informs gazebo to puslish tf in different topic
        ros_tf_static_remap.text = '/tf_static:=/' + args.robot_name + '/tf_static'

        tag_odom_frame = planar_move_plugin.find('odometry_frame')
        tag_odom_frame.text =  args.robot_name + '/' + tag_odom_frame.text
        tag_robot_base_frame = planar_move_plugin.find('robot_base_frame')
        robot_base_frame = tag_robot_base_frame.text
        tag_robot_base_frame.text = args.robot_name + '/' + tag_robot_base_frame.text

        print (ET.tostring(planar_move_plugin, encoding='unicode'))

        base_laser_front_plugin = None 
        for plugin in root.iter('plugin'):
            if 'gazebo_ros_base_laser_front_controller' in plugin.attrib.values():
                base_laser_front_plugin = plugin
                break

        # We change the namespace to the robots corresponding one
        laser_tag_ros_params = base_laser_front_plugin.find('ros')
        laser_tag_ns = ET.SubElement(laser_tag_ros_params, 'namespace')
        laser_tag_ns.text = '/' + args.robot_name

        laser_tf_frame_id = ET.SubElement(base_laser_front_plugin, 'frame_name')
        laser_tf_frame_id.text = tag_robot_base_frame.text #same as the robot_base_frame text base_link or base_footprint
        print ("LASER SCAN URDF : ROBOT BASER FRAME : ", tag_robot_base_frame.text)
        print ("LASER SCAN URDF : ", ET.tostring(base_laser_front_plugin, encoding='unicode'))

    # Set data for request
    request = SpawnEntity.Request()
    request.name = args.robot_name
    request.robot_namespace = args.robot_namespace
    request.xml = ET.tostring(root, encoding='unicode')
    request.initial_pose.position.x = float(args.x)
    request.initial_pose.position.y = float(args.y)
    request.initial_pose.position.z = float(args.z)


    if args.namespace is True:
        node.get_logger().info('Spawning `{}` on namespace `{}` at {}, {}, {}'.format(
            args.robot_name, args.robot_namespace, args.x, args.y, args.z))

        request.robot_namespace = args.robot_namespace
        print(args.robot_namespace)

    else:
        node.get_logger().info('Spawning `{}` at {}, {}, {}'.format(
            args.robot_name, args.x, args.y, args.z))

    node.get_logger().info('Spawning robot using service: `/spawn_entity`')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info('Done! Shutting down node.')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
