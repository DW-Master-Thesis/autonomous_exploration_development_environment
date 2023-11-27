#!/usr/bin/python3

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterDescriptor
from std_msgs.msg import Float32

mpl.rcParams['toolbar'] = 'None'
plt.ion()

time_duration = 0
first_iteration = True

explored_volume: np.ndarray = np.array([])
explored_volume_time: np.ndarray = np.array([])
traveling_distances: dict[np.ndarray] = {}
traveling_distances_time: dict[np.ndarray] = {}
run_times: dict[np.ndarray] = {}
run_times_time: dict[np.ndarray] = {}


def timeDurationCallback(msg):
  global time_duration, first_iteration
  if first_iteration == True:
    first_iteration = False
  time_duration = msg.data

def exploredVolumeCallback(msg):
  global explored_volume, explored_volume_time, time_duration
  explored_volume = np.append(explored_volume, msg.data)
  explored_volume_time = np.append(explored_volume_time, time_duration)

def create_run_time_callback(robot_name):
  def run_time_callback(msg):
    global run_times, run_times_time, time_duration
    run_times[robot_name] = np.append(run_times[robot_name], msg.data)
    run_times_time[robot_name] = np.append(run_times_time[robot_name], time_duration)
  return run_time_callback

def create_traveling_distance_callback(robot_name):
  def traveling_distance_callback(msg):
    global traveling_distances, traveling_distances_time, time_duration
    traveling_distances[robot_name] = np.append(traveling_distances[robot_name], msg.data)
    traveling_distances_time[robot_name] = np.append(traveling_distances_time[robot_name], time_duration)
  return traveling_distance_callback

class Listener(Node):

  def __init__(self):
    super().__init__('realTimePlot')

    self._robot_names: list[str] = []
    self.declare_parameter("robot_names", ["robot_1"])
    self._robot_names = self.get_parameter("robot_names").value

    self._init_data()
    self._init_figure()
    self._init_subscriptions()
    self.timer = self.create_timer(0.5, self.plot_callback)

  def _init_data(self):
    global traveling_distances, traveling_distances_time, run_times, run_times_time
    for robot_name in self._robot_names:
      traveling_distances[robot_name] = np.array([])
      traveling_distances_time[robot_name] = np.array([])
      run_times[robot_name] = np.array([])
      run_times_time[robot_name] = np.array([])

  def _init_figure(self):
    self.fig, (self.fig1, self.fig2, self.fig3) = plt.subplots(3, 1, sharex=True, figsize=(8,7))
    self.fig.suptitle("Exploration Metrics\n", fontsize=14)
    plt.margins(x=0.001)
    self.fig1.set_ylabel("Explored\nVolume (m$^3$)", fontsize=12)
    self._explored_volume_line = self.fig1.plot(
      explored_volume_time, explored_volume, color='r', label='Explored Volume'
    )[0]
    self.fig2.set_ylabel("Traveling\nDistance (m)", fontsize=12)
    self._traveled_distance_lines = {
      robot_name: self.fig2.plot(
        traveling_distances_time[robot_name], traveling_distances[robot_name], label=robot_name
      )[0]
      for robot_name in self._robot_names
    }
    self.fig3.set_ylabel("Algorithm\nRuntime (s)", fontsize=12)
    self.fig3.set_xlabel("Time Duration (s)", fontsize=12) #only set once
    self._run_time_lines = {
      robot_name: self.fig3.plot(
        run_times_time[robot_name], run_times[robot_name], label=robot_name
      )[0]
      for robot_name in self._robot_names
    }
    self.fig.legend(handles=self._run_time_lines.values(), loc='upper right', bbox_to_anchor=(1.0, 0.5))

  def _init_subscriptions(self):
    time_duration_topic = f"{self._robot_names[0]}/time_duration" if len(self._robot_names) == 1 else "time_duration"
    self.time_duration_subscriptions = self.create_subscription(
      Float32,
      time_duration_topic,
      timeDurationCallback,
      10,
    )

    explored_volume_topic = f"{self._robot_names[0]}/explored_volume" if len(self._robot_names) == 1 else "explored_volume"
    self.explored_volume_subscription = self.create_subscription(
      Float32,
      explored_volume_topic,
      exploredVolumeCallback,
      10,
    )

    self.runtime_subscriptions = []
    self.traveling_distance_subscriptions = []
    for robot_name in self._robot_names:
      runTimeCallback = create_run_time_callback(robot_name)
      travelingDistanceCallback = create_traveling_distance_callback(robot_name)
      self.runtime_subscriptions.append(
        self.create_subscription(
          Float32,
          f"{robot_name}/runtime",
          runTimeCallback,
          10,
        )
      )
      self.traveling_distance_subscriptions.append(
        self.create_subscription(
          Float32,
          f"{robot_name}/traveling_distance",
          travelingDistanceCallback,
          10,
        )
      )

  def plot_callback(self):
    global explored_volume, explored_volume_time, traveling_distances, traveling_distances_time, run_times, run_times_time, time_duration

    self._explored_volume_line.set_xdata(explored_volume_time)
    self._explored_volume_line.set_ydata(explored_volume)
    for robot_name in self._robot_names:
      self._traveled_distance_lines[robot_name].set_xdata(traveling_distances_time[robot_name])
      self._traveled_distance_lines[robot_name].set_ydata(traveling_distances[robot_name])
      self._run_time_lines[robot_name].set_xdata(run_times_time[robot_name])
      self._run_time_lines[robot_name].set_ydata(run_times[robot_name])

    max_explored_volume = 0 if explored_volume.size == 0 else max(explored_volume)
    max_traveling_distance = [max(traveling_distances[robot_name]) for robot_name in self._robot_names if traveling_distances[robot_name].size > 0]
    max_traveling_distance = 0 if len(max_traveling_distance) == 0 else max(max_traveling_distance)
    max_run_time = [max(run_times[robot_name]) for robot_name in self._robot_names if run_times[robot_name].size > 0]
    max_run_time = 0 if len(max_run_time) == 0 else max(max_run_time)
    self.fig1.set_ylim(0, max_explored_volume + 500)
    self.fig2.set_ylim(0, max_traveling_distance + 20)
    self.fig3.set_ylim(0, max_run_time + 0.2)
    self.fig1.set_xlim(0, time_duration + 10)
    self.fig2.set_xlim(0, time_duration + 10)
    self.fig3.set_xlim(0, time_duration + 10)
    plt.pause(0.001)


def main(args=None):
  rclpy.init(args=args)

  listener = Listener()

  rclpy.spin(listener)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  listener.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
