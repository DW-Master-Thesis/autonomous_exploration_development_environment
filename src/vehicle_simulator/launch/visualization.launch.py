import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch_ros.actions import Node, PushRosNamespace
from launch.substitutions import LaunchConfiguration 



def generate_launch_description():
  world_name = LaunchConfiguration('world_name')
  namespace = LaunchConfiguration('namespace')

  declare_world_name = DeclareLaunchArgument('world_name', default_value="indoor", description='')
  declare_namespace = DeclareLaunchArgument('namespace', default_value='/', description='')

  start_visualization_tools = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
    ),
    launch_arguments={
      'world_name': world_name,
    }.items()
  )

  rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'vehicle_simulator.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen',
    namespace=namespace,
  )

  delayed_start_rviz = TimerAction(
    period=8.0,
    actions=[
      start_rviz
    ]
  )

  # Add actions
  ld = LaunchDescription()
  ld.add_action(declare_world_name)
  ld.add_action(declare_namespace)

  start_actions_with_ns = GroupAction(
    actions=[
      PushRosNamespace(namespace=namespace),
        start_visualization_tools,
    ]
  )

  ld.add_action(start_actions_with_ns)
  ld.add_action(delayed_start_rviz)
  
  return ld
