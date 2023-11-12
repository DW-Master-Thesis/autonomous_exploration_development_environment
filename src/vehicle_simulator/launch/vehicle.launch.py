import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration


def spawn_robot_system(context, robotName, gazeboTimeout):
  robot_name_str = str(robotName.perform(context))
  spawn_lidar = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=[
      '-robot_namespace', robotName,
      '-timeout', gazeboTimeout,
      '-entity', robot_name_str + '_lidar',
      '-topic', 'robot_description',
    ],
    output='screen',
  )

  robot_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'robot.sdf')
  spawn_robot = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=[
      '-robot_namespace', robotName,
      '-timeout', gazeboTimeout,
      '-file', robot_xacro,
      '-entity', robot_name_str + '_robot'
    ],
    output='screen',
  )

  camera_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'camera.urdf.xacro')
  spawn_camera = Node(
    package='gazebo_ros', 
    executable='spawn_entity.py',
    arguments=[
      '-robot_namespace', robotName,
      '-timeout', gazeboTimeout,
      '-file', camera_xacro,
      '-entity', robot_name_str + '_camera'
      ],
      output='screen'
  )
  return [spawn_lidar, spawn_robot, spawn_camera]


def generate_launch_description():
  robotName = LaunchConfiguration('robotName')
  sensorOffsetX = LaunchConfiguration('sensorOffsetX')
  sensorOffsetY = LaunchConfiguration('sensorOffsetY')
  vehicleHeight = LaunchConfiguration('vehicleHeight')
  cameraOffsetZ = LaunchConfiguration('cameraOffsetZ')
  vehicleX = LaunchConfiguration('vehicleX')
  vehicleY = LaunchConfiguration('vehicleY')
  vehicleZ = LaunchConfiguration('vehicleZ')
  terrainZ = LaunchConfiguration('terrainZ')
  vehicleYaw = LaunchConfiguration('vehicleYaw')
  terrainVoxelSize = LaunchConfiguration('terrainVoxelSize')
  groundHeightThre = LaunchConfiguration('groundHeightThre')
  adjustZ = LaunchConfiguration('adjustZ')
  terrainRadiusZ = LaunchConfiguration('terrainRadiusZ')
  minTerrainPointNumZ = LaunchConfiguration('minTerrainPointNumZ')
  smoothRateZ = LaunchConfiguration('smoothRateZ')
  adjustIncl = LaunchConfiguration('adjustIncl')
  terrainRadiusIncl = LaunchConfiguration('terrainRadiusIncl')
  minTerrainPointNumIncl = LaunchConfiguration('minTerrainPointNumIncl')
  smoothRateIncl = LaunchConfiguration('smoothRateIncl')
  InclFittingThre = LaunchConfiguration('InclFittingThre')
  maxIncl = LaunchConfiguration('maxIncl')
  pause = LaunchConfiguration('pause')
  use_sim_time = LaunchConfiguration('use_sim_time')
  gazeboTimeout = LaunchConfiguration('gazeboTimeout')
  record = LaunchConfiguration('record')
  verbose = LaunchConfiguration('verbose')

  declare_robotName = DeclareLaunchArgument('robotName', default_value='/', description='')
  declare_sensorOffsetX = DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description='')
  declare_sensorOffsetY = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_vehicleHeight = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
  declare_cameraOffsetZ = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicleX = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicleY = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_vehicleZ = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
  declare_terrainZ = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
  declare_vehicleYaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
  declare_terrainVoxelSize = DeclareLaunchArgument('terrainVoxelSize', default_value='0.05', description='')
  declare_groundHeightThre = DeclareLaunchArgument('groundHeightThre', default_value='0.1', description='')
  declare_adjustZ = DeclareLaunchArgument('adjustZ', default_value='true', description='')
  declare_terrainRadiusZ = DeclareLaunchArgument('terrainRadiusZ', default_value='1.0', description='')
  declare_minTerrainPointNumZ = DeclareLaunchArgument('minTerrainPointNumZ', default_value='5', description='')
  declare_smoothRateZ = DeclareLaunchArgument('smoothRateZ', default_value='0.5', description='')
  declare_adjustIncl = DeclareLaunchArgument('adjustIncl', default_value='true', description='')
  declare_terrainRadiusIncl = DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0', description='')
  declare_minTerrainPointNumIncl = DeclareLaunchArgument('minTerrainPointNumIncl', default_value='200', description='')
  declare_smoothRateIncl = DeclareLaunchArgument('smoothRateIncl', default_value='0.5', description='')
  declare_InclFittingThre = DeclareLaunchArgument('InclFittingThre', default_value='0.2', description='')
  declare_maxIncl = DeclareLaunchArgument('maxIncl', default_value='30.0', description='')
  declare_pause = DeclareLaunchArgument('pause', default_value='false', description='')
  declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='')
  declare_gazeboTimeout = DeclareLaunchArgument('gazeboTimeout', default_value='30', description='')
  declare_record = DeclareLaunchArgument('record', default_value='false', description='')
  declare_verbose = DeclareLaunchArgument('verbose', default_value='false', description='')
  
  lidar_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'lidar.urdf.xacro')
  lidar_description = Command(['xacro',' ', lidar_xacro])
  start_lidar_state_publisher = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
      'use_sim_time': use_sim_time,
      'robot_description': lidar_description
    }]
  )

  start_vehicle_simulator = Node(
    package='vehicle_simulator', 
    executable='vehicleSimulator',
    namespace=robotName,
    parameters=[
      {
        'vehicleName': robotName,
        'use_gazebo_time': False,
        'sensorOffsetX': sensorOffsetX,
        'sensorOffsetY': sensorOffsetY,
        'vehicleHeight': vehicleHeight,
        'cameraOffsetZ': cameraOffsetZ,
        'vehicleX': vehicleX,
        'vehicleY': vehicleY,
        'vehicleZ': vehicleZ,
        'terrainZ': terrainZ,
        'vehicleYaw': vehicleYaw,
        'terrainVoxelSize': terrainVoxelSize,
        'groundHeightThre': groundHeightThre,
        'adjustZ': adjustZ,
        'terrainRadiusZ': terrainRadiusZ,
        'minTerrainPointNumZ': minTerrainPointNumZ,
        'smoothRateZ': smoothRateZ,
        'adjustIncl': adjustIncl,
        'terrainRadiusIncl': terrainRadiusIncl,
        'minTerrainPointNumIncl': minTerrainPointNumIncl,
        'smoothRateIncl': smoothRateIncl,
        'InclFittingThre': InclFittingThre,
        'maxIncl': maxIncl,
        'use_sim_time': use_sim_time
      }
      ],
      output='screen'
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_robotName)
  ld.add_action(declare_sensorOffsetX)
  ld.add_action(declare_sensorOffsetY)
  ld.add_action(declare_vehicleHeight)
  ld.add_action(declare_cameraOffsetZ)
  ld.add_action(declare_vehicleX)
  ld.add_action(declare_vehicleY)
  ld.add_action(declare_vehicleZ)
  ld.add_action(declare_terrainZ)
  ld.add_action(declare_vehicleYaw)
  ld.add_action(declare_terrainVoxelSize)
  ld.add_action(declare_groundHeightThre)
  ld.add_action(declare_adjustZ)
  ld.add_action(declare_terrainRadiusZ)
  ld.add_action(declare_minTerrainPointNumZ)
  ld.add_action(declare_smoothRateZ)
  ld.add_action(declare_adjustIncl)
  ld.add_action(declare_terrainRadiusIncl)
  ld.add_action(declare_minTerrainPointNumIncl)
  ld.add_action(declare_smoothRateIncl)
  ld.add_action(declare_InclFittingThre)
  ld.add_action(declare_maxIncl)
  ld.add_action(declare_pause)
  ld.add_action(declare_use_sim_time)
  ld.add_action(declare_gazeboTimeout)
  ld.add_action(declare_record)
  ld.add_action(declare_verbose)

  ld.add_action(start_lidar_state_publisher)
  ld.add_action(OpaqueFunction(function=spawn_robot_system, args=[robotName, gazeboTimeout]))
  ld.add_action(start_vehicle_simulator)

  return ld
