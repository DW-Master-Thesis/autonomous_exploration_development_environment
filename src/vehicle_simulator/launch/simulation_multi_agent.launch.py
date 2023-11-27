import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import Node


def get_default_agents_config_filepath():
  return os.path.join(get_package_share_directory('vehicle_simulator'), 'config', 'agents.yaml')


def start_vehicles_and_visualization_tools(context, agentsConfigFile, gazeboTimeout, checkTerrainConn, worldName):
  agents_config_filepath = str(agentsConfigFile.perform(context))
  launch_descriptions = []
  robots_config = yaml.safe_load(open(agents_config_filepath))
  robot_names = list(robots_config.keys())
  for robot_name, robot_config in robots_config.items():
    for key, value in robot_config.items():
      robot_config[key] = str(value)
    start_vehicle = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(os.path.join(
        get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle_with_planner.launch.py')
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
        'gazeboTimeout': gazeboTimeout,
        'checkTerrainConn': checkTerrainConn,
      }.items()
    )
    start_visualization_tools = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(os.path.join(
        get_package_share_directory('visualization_tools'), 'launch', 'visualization_tools.launch')
      ),
      launch_arguments={
        'worldName': worldName,
        'robotName': robot_name,
      }.items()
    )
    launch_descriptions.extend([start_vehicle, start_visualization_tools])

  start_multi_agent_visualization_tools = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('visualization_tools'), 'launch', 'multi_agent_visualization_tools.launch.xml')
    ),
    launch_arguments={
      'robotNames': yaml.dump(robot_names),
      'worldName': worldName,
    }.items()
  )
  launch_descriptions.append(start_multi_agent_visualization_tools)

  start_real_time_plot = Node(
    package='visualization_tools',
    executable='realTimePlot.py',
    name='realTimePlot',
    output='screen',
    parameters=[
      {'robot_names':  robot_names},
    ],
  )
  launch_descriptions.append(start_real_time_plot)

  return launch_descriptions


def generate_launch_description():
  agentsConfigFile = LaunchConfiguration('agentsConfigFile')
  worldName = LaunchConfiguration('worldName')
  gazeboTimeout = LaunchConfiguration('gazeboTimeout')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  gazebo_gui = LaunchConfiguration('gazebo_gui')
  
  declare_agentsConfigFile = DeclareLaunchArgument(
    'agentsConfigFile',
    default_value=get_default_agents_config_filepath(),
    description='',
  )
  declare_worldName = DeclareLaunchArgument('worldName', default_value="indoor", description='')
  declare_gazeboTimeout = DeclareLaunchArgument('gazeboTimeout', default_value='60.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='false', description='')
  declare_gazebo_gui = DeclareLaunchArgument('gazebo_gui', default_value='false', description='')

  start_gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('vehicle_simulator'), 'launch', 'gazebo.launch.py')
    ),
    launch_arguments={
      'worldName': worldName,
      'gui': gazebo_gui,
    }.items()
  )

  rviz_config_file = os.path.join(get_package_share_directory('vehicle_simulator'), 'rviz', 'multi_agent_simulator.rviz')
  start_rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', rviz_config_file],
    output='screen',
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_agentsConfigFile)
  ld.add_action(declare_worldName)
  ld.add_action(declare_gazeboTimeout)
  ld.add_action(declare_checkTerrainConn)
  ld.add_action(declare_gazebo_gui)

  ld.add_action(start_gazebo)
  ld.add_action(OpaqueFunction(
    function=start_vehicles_and_visualization_tools,
    args=[agentsConfigFile, gazeboTimeout, checkTerrainConn, worldName],
  ))
  ld.add_action(start_rviz)

  return ld
