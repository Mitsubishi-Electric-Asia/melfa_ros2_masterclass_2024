#    COPYRIGHT (C) 2024 Mitsubishi Electric Corporation

#    Licensed under the Apache License, Version 2.0 (the "License");
#    you may not use this file except in compliance with the License.
#    You may obtain a copy of the License at

#        http://www.apache.org/licenses/LICENSE-2.0

#    Unless required by applicable law or agreed to in writing, software
#    distributed under the License is distributed on an "AS IS" BASIS,
#    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#    See the License for the specific language governing permissions and
#    limitations under the License.
#    Contributor(s):
#       Liu Muyao

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import os
import yaml

def generate_launch_description():

    config = os.path.join(
      get_package_share_directory('melfa_rv5as_masterclass'),
      'config',
      'rv5as_params.yaml'
      )

    moveit_config = MoveItConfigsBuilder("rv5as", package_name="melfa_rv5as_moveit_config").to_moveit_configs()
    pick_n_place = Node(
        package="melfa_rv5as_masterclass",
        executable="pick_n_place_",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            config
        ],
    )
    return LaunchDescription([pick_n_place])
