# pylint: disable=missing-module-docstring,missing-function-docstring
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration


def spawn_robot_system(context):
  robot_name = str(LaunchConfiguration('robotName').perform(context))
  gazebo_timeout = str(LaunchConfiguration('gazeboTimeout').perform(context))
  use_sim_time = str(LaunchConfiguration('use_sim_time').perform(context))

  lidar_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'lidar.urdf.xacro')
  lidar_description = Command(['xacro', ' ', lidar_xacro])
  start_lidar_state_publisher = Node(
      package='robot_state_publisher',
      executable='robot_state_publisher',
      name='robot_state_publisher',
      output='screen',
      parameters=[{
          'use_sim_time': use_sim_time,
          'robot_description': lidar_description,
          'tf_prefix': robot_name,
      }],
      namespace=robot_name,
  )

  spawn_lidar = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
          '-robot_namespace',
          robot_name,
          '-timeout',
          gazebo_timeout,
          '-entity',
          robot_name + '_lidar',
          '-topic',
          robot_name + '/robot_description',
      ],
      output='screen',
  )

  robot_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'robot.sdf')
  spawn_robot = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
          '-robot_namespace', robot_name, '-timeout', gazebo_timeout, '-file', robot_xacro, '-entity',
          robot_name + '_robot'
      ],
      output='screen',
  )

  camera_xacro = os.path.join(get_package_share_directory('vehicle_simulator'), 'urdf', 'camera.urdf.xacro')
  spawn_camera = Node(
      package='gazebo_ros',
      executable='spawn_entity.py',
      arguments=[
          '-robot_namespace', robot_name, '-timeout', gazebo_timeout, '-file', camera_xacro, '-entity',
          robot_name + '_camera'
      ],
      output='screen'
  )
  return [spawn_lidar, spawn_robot, spawn_camera, start_lidar_state_publisher]


def generate_launch_description():  # pylint: disable=too-many-statements
  declare_robot_name = DeclareLaunchArgument('robotName', default_value='/', description='')
  declare_sensor_offset_x = DeclareLaunchArgument('sensorOffsetX', default_value='0.0', description='')
  declare_sensor_offset_y = DeclareLaunchArgument('sensorOffsetY', default_value='0.0', description='')
  declare_vehicle_height = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
  declare_camera_offset_z = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicle_x = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicle_y = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_vehicle_z = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
  declare_terrain_z = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
  declare_vehicle_yaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
  declare_terrain_voxel_size = DeclareLaunchArgument('terrainVoxelSize', default_value='0.05', description='')
  declare_ground_height_thre = DeclareLaunchArgument('groundHeightThre', default_value='0.1', description='')
  declare_adjust_z = DeclareLaunchArgument('adjustZ', default_value='true', description='')
  declare_terrain_radius_z = DeclareLaunchArgument('terrainRadiusZ', default_value='1.0', description='')
  declare_min_terrain_point_num_z = DeclareLaunchArgument('minTerrainPointNumZ', default_value='5', description='')
  declare_smooth_rate_z = DeclareLaunchArgument('smoothRateZ', default_value='0.5', description='')
  declare_adjust_incl = DeclareLaunchArgument('adjustIncl', default_value='true', description='')
  declare_terrain_radius_incl = DeclareLaunchArgument('terrainRadiusIncl', default_value='2.0', description='')
  declare_min_terrain_point_num_incl = DeclareLaunchArgument(
      'minTerrainPointNumIncl', default_value='200', description=''
  )
  declare_smooth_rate_incl = DeclareLaunchArgument('smoothRateIncl', default_value='0.5', description='')
  declare_incl_fitting_thre = DeclareLaunchArgument('InclFittingThre', default_value='0.2', description='')
  declare_max_incl = DeclareLaunchArgument('maxIncl', default_value='30.0', description='')
  declare_pause = DeclareLaunchArgument('pause', default_value='false', description='')
  declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false', description='')
  declare_gazebo_timeout = DeclareLaunchArgument('gazeboTimeout', default_value='30', description='')
  declare_record = DeclareLaunchArgument('record', default_value='false', description='')
  declare_verbose = DeclareLaunchArgument('verbose', default_value='false', description='')

  start_vehicle_simulator = Node(
      package='vehicle_simulator',
      executable='vehicleSimulator',
      namespace=LaunchConfiguration("robotName"),
      parameters=[{
          'vehicleName': LaunchConfiguration("robotName"),
          'use_gazebo_time': False,
          'sensorOffsetX': LaunchConfiguration("sensorOffsetX"),
          'sensorOffsetY': LaunchConfiguration("sensorOffsetY"),
          'vehicleHeight': LaunchConfiguration("vehicleHeight"),
          'cameraOffsetZ': LaunchConfiguration("cameraOffsetZ"),
          'vehicleX': LaunchConfiguration("vehicleX"),
          'vehicleY': LaunchConfiguration("vehicleY"),
          'vehicleZ': LaunchConfiguration("vehicleZ"),
          'terrainZ': LaunchConfiguration("terrainZ"),
          'vehicleYaw': LaunchConfiguration("vehicleYaw"),
          'terrainVoxelSize': LaunchConfiguration("terrainVoxelSize"),
          'groundHeightThre': LaunchConfiguration("groundHeightThre"),
          'adjustZ': LaunchConfiguration("adjustZ"),
          'terrainRadiusZ': LaunchConfiguration("terrainRadiusZ"),
          'minTerrainPointNumZ': LaunchConfiguration("minTerrainPointNumZ"),
          'smoothRateZ': LaunchConfiguration("smoothRateZ"),
          'adjustIncl': LaunchConfiguration("adjustIncl"),
          'terrainRadiusIncl': LaunchConfiguration("terrainRadiusIncl"),
          'minTerrainPointNumIncl': LaunchConfiguration("minTerrainPointNumIncl"),
          'smoothRateIncl': LaunchConfiguration("smoothRateIncl"),
          'InclFittingThre': LaunchConfiguration("InclFittingThre"),
          'maxIncl': LaunchConfiguration("maxIncl"),
          'use_sim_time': LaunchConfiguration("use_sim_time")
      }],
      output='screen'
  )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_robot_name)
  ld.add_action(declare_sensor_offset_x)
  ld.add_action(declare_sensor_offset_y)
  ld.add_action(declare_vehicle_height)
  ld.add_action(declare_camera_offset_z)
  ld.add_action(declare_vehicle_x)
  ld.add_action(declare_vehicle_y)
  ld.add_action(declare_vehicle_z)
  ld.add_action(declare_terrain_z)
  ld.add_action(declare_vehicle_yaw)
  ld.add_action(declare_terrain_voxel_size)
  ld.add_action(declare_ground_height_thre)
  ld.add_action(declare_adjust_z)
  ld.add_action(declare_terrain_radius_z)
  ld.add_action(declare_min_terrain_point_num_z)
  ld.add_action(declare_smooth_rate_z)
  ld.add_action(declare_adjust_incl)
  ld.add_action(declare_terrain_radius_incl)
  ld.add_action(declare_min_terrain_point_num_incl)
  ld.add_action(declare_smooth_rate_incl)
  ld.add_action(declare_incl_fitting_thre)
  ld.add_action(declare_max_incl)
  ld.add_action(declare_pause)
  ld.add_action(declare_use_sim_time)
  ld.add_action(declare_gazebo_timeout)
  ld.add_action(declare_record)
  ld.add_action(declare_verbose)

  ld.add_action(OpaqueFunction(function=spawn_robot_system))
  ld.add_action(start_vehicle_simulator)

  return ld
