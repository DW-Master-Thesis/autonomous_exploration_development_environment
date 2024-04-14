# pylint: disable=missing-module-docstring,missing-function-docstring
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import (FrontendLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration


def get_default_agents_config_filepath():
  return os.path.join(get_package_share_directory('vehicle_simulator'), 'config', 'agents.yaml')


def start_vehicles_and_visualization_tools(context):
  agents_config_filepath = str(LaunchConfiguration("agentsConfigFile").perform(context))
  namespace = str(LaunchConfiguration("namespace").perform(context))
  launch_descriptions = []
  with open(agents_config_filepath, 'r', encoding="utf-8") as file:
    robots_config = yaml.safe_load(file)
  robot_names = list(robots_config.keys())
  robot_names = [namespace + '/' + name for name in robot_names]
  for robot_name, robot_config in robots_config.items():
    robot_name = namespace + '/' + robot_name
    for key, value in robot_config.items():
      robot_config[key] = str(value)
    start_vehicle = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle_with_planner.launch.py')
        ),
        launch_arguments={
            'robotName': robot_name,
            'vehicleHeight': robot_config["vehicleHeight"],
            'cameraOffsetZ': robot_config["cameraOffsetZ"],
            'vehicleX': robot_config["vehicleX"],
            'vehicleY': robot_config["vehicleY"],
            'vehicleZ': robot_config["vehicleZ"],
            'terrainZ': robot_config["terrainZ"],
            'vehicleYaw': robot_config["vehicleYaw"],
            'gazeboTimeout': LaunchConfiguration("gazeboTimeout"),
            'checkTerrainConn': LaunchConfiguration("checkTerrainConn"),
        }.items()
    )
    start_visualization_tools = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
        ),
        launch_arguments={
            'worldName': LaunchConfiguration("worldName"),
            'robotName': robot_name,
        }.items()
    )
    launch_descriptions.extend([start_vehicle, start_visualization_tools])

  start_multi_agent_visualization_tools = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(
              get_package_share_directory('visualization_tools'), 'launch', 'multi_agent_visualization_tools.launch.xml'
          )
      ),
      launch_arguments={
          'robotNames': yaml.dump(robot_names),
          'worldName': LaunchConfiguration("worldName"),
          'namespace': namespace,
      }.items(),
  )
  launch_descriptions.append(start_multi_agent_visualization_tools)

  start_real_time_plot = Node(
      package='visualization_tools',
      executable='realTimePlot.py',
      name='realTimePlot',
      output='screen',
      parameters=[
          {
              'robot_names': robot_names
          },
      ],
      condition=IfCondition(LaunchConfiguration("use_rviz")),
  )
  launch_descriptions.append(start_real_time_plot)

  return launch_descriptions


def generate_launch_description():
  declare_agents_config_file = DeclareLaunchArgument(
      'agentsConfigFile',
      default_value=get_default_agents_config_filepath(),
      description='',
  )
  declare_world_name = DeclareLaunchArgument('worldName', default_value="indoor", description='')
  declare_gazebo_timeout = DeclareLaunchArgument('gazeboTimeout', default_value='60.0', description='')
  declare_check_terrain_conn = DeclareLaunchArgument('checkTerrainConn', default_value='false', description='')
  declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='false', description='')
  declare_use_rviz = DeclareLaunchArgument('use_rviz', default_value='false', description='')
  declare_namespace = DeclareLaunchArgument('namespace', default_value='default', description='')

  rviz_config_file = os.path.join(
      get_package_share_directory('vehicle_simulator'), 'rviz', 'multi_agent_simulator.rviz'
  )
  start_rviz = Node(
      package='rviz2',
      executable='rviz2',
      arguments=['-d', rviz_config_file],
      output='screen',
      condition=IfCondition(LaunchConfiguration("use_rviz")),
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_agents_config_file)
  ld.add_action(declare_world_name)
  ld.add_action(declare_gazebo_timeout)
  ld.add_action(declare_check_terrain_conn)
  ld.add_action(declare_gazebo_gui)
  ld.add_action(declare_namespace)
  ld.add_action(declare_use_rviz)

  ld.add_action(OpaqueFunction(function=start_vehicles_and_visualization_tools))
  ld.add_action(start_rviz)

  return ld
