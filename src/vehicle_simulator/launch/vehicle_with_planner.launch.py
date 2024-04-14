# pylint: disable=missing-module-docstring,missing-function-docstring
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import PushRosNamespace
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction,
                            IncludeLaunchDescription)
from launch.launch_description_sources import (FrontendLaunchDescriptionSource,
                                               PythonLaunchDescriptionSource)
from launch.substitutions import LaunchConfiguration


def generate_launch_description():  # pylint: disable=too-many-locals
  declare_robot_name = DeclareLaunchArgument('robotName', default_value='/', description='')
  declare_vehicle_height = DeclareLaunchArgument('vehicleHeight', default_value='0.75', description='')
  declare_camera_offest_z = DeclareLaunchArgument('cameraOffsetZ', default_value='0.0', description='')
  declare_vehicle_x = DeclareLaunchArgument('vehicleX', default_value='0.0', description='')
  declare_vehicle_y = DeclareLaunchArgument('vehicleY', default_value='0.0', description='')
  declare_vehicle_z = DeclareLaunchArgument('vehicleZ', default_value='0.0', description='')
  declare_terrain_z = DeclareLaunchArgument('terrainZ', default_value='0.0', description='')
  declare_vehicle_yaw = DeclareLaunchArgument('vehicleYaw', default_value='0.0', description='')
  declare_gazebo_timeout = DeclareLaunchArgument('gazeboTimeout', default_value='60.0', description='')
  declare_check_terrain_conn = DeclareLaunchArgument('checkTerrainConn', default_value='false', description='')

  start_local_planner = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory('local_planner'), 'launch', 'local_planner.launch')
      ),
      launch_arguments={
          'robotName': LaunchConfiguration("robotName"),
          'cameraOffsetZ': LaunchConfiguration("cameraOffsetZ"),
          'goalX': LaunchConfiguration("vehicleX"),
          'goalY': LaunchConfiguration("vehicleY"),
      }.items()
  )

  start_terrain_analysis = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory('terrain_analysis'), 'launch', 'terrain_analysis.launch')
      )
  )

  start_terrain_analysis_ext = IncludeLaunchDescription(
      FrontendLaunchDescriptionSource(
          os.path.join(get_package_share_directory('terrain_analysis_ext'), 'launch', 'terrain_analysis_ext.launch')
      ),
      launch_arguments={
          'checkTerrainConn': LaunchConfiguration("checkTerrainConn"),
      }.items()
  )

  start_vehicle_simulator = IncludeLaunchDescription(
      PythonLaunchDescriptionSource(
          os.path.join(get_package_share_directory('vehicle_simulator'), 'launch', 'vehicle.launch.py')
      ),
      launch_arguments={
          'robotName': LaunchConfiguration("robotName"),
          'vehicleHeight': LaunchConfiguration("vehicleHeight"),
          'cameraOffsetZ': LaunchConfiguration("cameraOffsetZ"),
          'vehicleX': LaunchConfiguration("vehicleX"),
          'vehicleY': LaunchConfiguration("vehicleY"),
          'vehicleZ': LaunchConfiguration("vehicleZ"),
          'terrainZ': LaunchConfiguration("terrainZ"),
          'vehicleYaw': LaunchConfiguration("vehicleYaw"),
          'gazeboTimeout': LaunchConfiguration("gazeboTimeout"),
      }.items()
  )

  # start_sensor_scan_generation = IncludeLaunchDescription(
  #     FrontendLaunchDescriptionSource(
  #         os.path.join(
  #             get_package_share_directory('sensor_scan_generation'), 'launch', 'sensor_scan_generation.launch'
  #         )
  #     )
  # )

  ld = LaunchDescription()

  # Add the actions
  ld.add_action(declare_robot_name)
  ld.add_action(declare_vehicle_height)
  ld.add_action(declare_camera_offest_z)
  ld.add_action(declare_vehicle_x)
  ld.add_action(declare_vehicle_y)
  ld.add_action(declare_vehicle_z)
  ld.add_action(declare_terrain_z)
  ld.add_action(declare_vehicle_yaw)
  ld.add_action(declare_gazebo_timeout)
  ld.add_action(declare_check_terrain_conn)

  ld.add_action(start_local_planner)
  start_actions_with_ns = GroupAction(
      actions=[
          PushRosNamespace(namespace=LaunchConfiguration("robotName")),
          start_terrain_analysis,
          start_terrain_analysis_ext,
          # start_sensor_scan_generation,
      ]
  )
  ld.add_action(start_vehicle_simulator)
  ld.add_action(start_actions_with_ns)

  return ld
