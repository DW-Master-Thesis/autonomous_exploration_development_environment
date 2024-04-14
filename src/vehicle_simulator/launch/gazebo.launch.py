# pylint: disable=missing-module-docstring,missing-function-docstring
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def declare_world_action(context):
  world_name = str(LaunchConfiguration('worldName').perform(context))
  declare_world = DeclareLaunchArgument(
      'world',
      default_value=[
          os.path.join(get_package_share_directory('vehicle_simulator'), 'world', world_name + '.world')
      ],
      description=''
  )
  return [declare_world]


def generate_launch_description():
  declare_world_name = DeclareLaunchArgument('worldName', default_value='garage', description='')
  declare_gui = DeclareLaunchArgument('gui', default_value='false', description='')

  start_gazebo = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
      ),
  )

  ld = LaunchDescription()
  ld.add_action(declare_world_name)
  ld.add_action(declare_gui)

  ld.add_action(OpaqueFunction(function=declare_world_action))
  ld.add_action(start_gazebo)

  return ld
