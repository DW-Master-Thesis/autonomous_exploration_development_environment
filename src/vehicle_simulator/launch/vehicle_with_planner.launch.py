import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource, FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration 
from launch_ros.actions import PushRosNamespace

def generate_launch_description():
  robotName = LaunchConfiguration('robotName')
  vehicleHeight = LaunchConfiguration('vehicleHeight')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  vehicleZ = LaunchConfiguration('vehicleZ')
  terrainZ = LaunchConfiguration('terrainZ')
  vehicleYaw = LaunchConfiguration('vehicleYaw')
  gazeboTimeout = LaunchConfiguration('gazeboTimeout')
  checkTerrainConn = LaunchConfiguration('checkTerrainConn')
  
  declare_robotName = DeclareLaunchArgument('robotName', default_value='/', description='')
  declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_vehicleZ = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
  declare_terrainZ = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
  declare_vehicleYaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
  declare_gazeboTimeout = DeclareLaunchArgument('gazeboTimeout', default_value='60.0', description='')
  declare_checkTerrainConn = DeclareLaunchArgument('checkTerrainConn', default_value='false', description='')
  
  start_local_planner = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
    ),
    launch_arguments={
      'cameraOffsetZ': cameraOffsetZ,
      'goalX': vehicleX,
      'goalY': vehicleY,
    }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
    )
  )

  start_terrain_analysis_ext = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
    ),
    launch_arguments={
      'checkTerrainConn': checkTerrainConn,
    }.items()
  )

  start_vehicle_simulator = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(os.path.join(
      get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle.launch.py')
    ),
    launch_arguments={
      'robotName': robotName,
      'vehicleHeight': vehicleHeight,
      'cameraOffsetZ': cameraOffsetZ,
      'vehicleX': vehicleX,
      'vehicleY': vehicleY,
      'vehicleZ': vehicleZ,
      'terrainZ': terrainZ,
      'vehicleYaw': vehicleYaw,
      'gazeboTimeout': gazeboTimeout,
    }.items()
  )

  start_sensor_scan_generation = IncludeLaunchDescription(
    FrontendLaunchDescriptionSource(os.path.join(
      get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch')
    )
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_robotName)
  ld.add_action(declare_vehicleHeight)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_vehicleZ)
  ld.add_action(declare_terrainZ)
  ld.add_action(declare_vehicleYaw)
  ld.add_action(declare_gazeboTimeout)
  ld.add_action(declare_checkTerrainConn)

  start_actions_with_ns = GroupAction(
    actions=[
      PushRosNamespace(namespace=robotName),
      start_local_planner,
      start_terrain_analysis,
      start_terrain_analysis_ext,
      start_sensor_scan_generation,
    ]
  )
  ld.add_action(start_vehicle_simulator)
  ld.add_action(start_actions_with_ns)

  return ld
