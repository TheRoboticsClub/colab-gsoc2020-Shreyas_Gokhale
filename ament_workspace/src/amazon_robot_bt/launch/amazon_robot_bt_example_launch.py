# Copyright 2019 Intelligent Robotics Lab
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
# Modified By: Shreyas Gokhale <shreyas6gokhale@gmail.com>

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    example_dir = get_package_share_directory('amazon_robot_bt')
    namespace = LaunchConfiguration('namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace')

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_CONSOLE_STDOUT_LINE_BUFFERED', '1')


    lifecycle_nodes = ['move']


    # plansys2_cmd = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(os.path.join(
    #         get_package_share_directory('amazon_robot_bringup'),
    #         'launch',
    #         'plansys2_bringup_launch_monolithic.py')),
    #     launch_arguments={
    #       'model_file': example_dir + '/pddl/bt_example.pddl',
    #       'namespace': namespace
    #       }.items())

    # Specify the actions
    move_cmd = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='move',
        namespace=namespace,
        output='screen',
        parameters=[
          example_dir + '/config/params.yaml',
          {
            'action_name': 'move',
            'bt_xml_file': example_dir + '/behavior_trees_xml/move.xml'
          }
        ])

    nav2_cmd = Node(
            package='amazon_robot_bt',
            executable='nav2_sim_node',
            name='nav2_node',
            namespace=namespace,
            output='screen'
            )



    # transport_cmd = Node(
    #     package='plansys2_bt_actions',
    #     executable='bt_action_node',
    #     name='transport',
    #     namespace=namespace,
    #     output='screen',
    #     parameters=[
    #       example_dir + '/config/params.yaml',
    #       {
    #         'action_name': 'transport',
    #         'bt_xml_file': example_dir + '/behavior_trees_xml/transport.xml'
    #       }
    #     ])


    lifecycle_node_cmd  =  Node(
          package='nav2_lifecycle_manager',
          executable='lifecycle_manager',
          name='lifecycle_manager_navigation',
          output='screen',
          parameters=[{'use_sim_time': True},
                      {'autostart': True},
                      {'node_names': lifecycle_nodes}])



    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(declare_namespace_cmd)

    # Declare the launch options
    # ld.add_action(plansys2_cmd)

    ld.add_action(move_cmd)
    ld.add_action(nav2_cmd)
    ld.add_action(lifecycle_node_cmd)

    # ld.add_action(transport_cmd)
    # ld.add_action(assemble_cmd)

    return ld
