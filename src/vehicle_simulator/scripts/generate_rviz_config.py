import pathlib
import yaml

BASE_RVIZ_CONFIG_FILE = pathlib.Path(__file__).parent.parent / "rviz" / "multi_agent_simulator_base.rviz"
OUTPUT_RVIZ_CONFIG_FILE = pathlib.Path(__file__).parent.parent / "rviz" / "multi_agent_simulator.rviz"
AGENTS_CONFIG_FILE = pathlib.Path(__file__).parent.parent / "config" / "agents.yaml"


def main():
  rviz_config = load_yaml(BASE_RVIZ_CONFIG_FILE)
  agents_config = load_yaml(AGENTS_CONFIG_FILE)

  for agent_name in agents_config.keys():
    rviz_config = add_axes(agent_name, rviz_config)
    rviz_config = add_image(agent_name, rviz_config)
    rviz_config = add_waypoint(agent_name, rviz_config)
    rviz_config = add_path(agent_name, rviz_config)
    rviz_config = add_free_paths(agent_name, rviz_config)
    rviz_config = add_trajectory(agent_name, rviz_config)
    rviz_config = add_explored_area(agent_name, rviz_config)
    ColorManager.tick()

  save_yaml(OUTPUT_RVIZ_CONFIG_FILE, rviz_config)


def load_yaml(file_path: pathlib.Path) -> dict:
  with open(file_path, "r") as f:
    return yaml.safe_load(f)


def save_yaml(file_path: pathlib.Path, data: dict) -> None:
  with open(file_path, "w") as f:
    yaml.dump(data, f)


def add_axes(agent_name: str, rviz_config: dict) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Class": "rviz_default_plugins/Axes",
    "Enabled": True,
    "Length": 1,
    "Name": agent_name,
    "Radius": 0.1,
    "Reference Frame": agent_name,
    "Value": True,
  })
  return rviz_config


def add_image(agent_name: str, rviz_config: dict) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Class": "rviz_default_plugins/Image",
    "Enabled": True,
    "Max Value": 1,
    "Median window": 5,
    "Min Value": 0,
    "Name": f"{agent_name} Image",
    "Normalize Range": True,
    "Topic": {
        "Depth": 5,
        "Durability Policy": "Volatile",
        "History Policy": "Keep Last",
        "Reliability Policy": "Reliable",
        "Value": f"/{agent_name}/camera/image_raw",
      },
    "Value": True,
  })
  rviz_config["Window Geometry"][f"{agent_name} Image"] = {
    "collapsed": False,
  }
  return rviz_config


def add_waypoint(agent_name: str, rviz_config: dict) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Alpha": 1,
    "Class": "rviz_default_plugins/PointStamped",
    "Color": ColorManager.get_color(),
    "Enabled": True,
    "History Length": 1,
    "Name": f"{agent_name} Waypoint",
    "Radius": 1.2,
    "Topic": get_topic(agent_name, "way_point"),
    "Value": True,
  })
  return rviz_config


def add_path(agent_name: str, rviz_config: dict) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Alpha": 1,
    "Buffer Length": 1,
    "Class": "rviz_default_plugins/Path",
    "Color": ColorManager.get_color(),
    "Enabled": True,
    "Head Diameter": 0.3,
    "Head Length": 0.2,
    "Length": 0.3,
    "Line Style": "Billboards",
    "Line Width": 0.05,
    "Name": f"{agent_name} Path",
    "Offset": {"X": 0, "Y": 0, "Z": 0},
    "Pose Color": "255; 85; 255",
    "Pose Style": None,
    "Radius": 0.03,
    "Shaft Diameter": 0.1,
    "Shaft Length": 0.1,
    "Topic": get_topic(agent_name, "path"),
    "Vaue": True,
  })
  return rviz_config


def add_free_paths(agent_name: str, rviz_config: dict) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Alpha": 1,
    "Autocompute Intensity Bounds": False,
    "Autocompute Value Bounds": {
      "Max Value": 10,
      "Min Value": -10,
      "Value": True,
    },
    "Axis": "Z",
    "Channel Name": "intensity",
    "Class": "rviz_default_plugins/PointCloud2",
    "Color": ColorManager.get_color(),
    "Color Transformer": "Intensity",
    "Decay Time": 0,
    "Enabled": True,
    "Invert Rainbow": False,
    "Max Color": "255; 255; 255",
    "Max Intensity": 6,
    "Min Color": "0; 0; 0",
    "Min Intensity": 0,
    "Name": f"{agent_name} Free Paths",
    "Position Transformer": "XYZ",
    "Selectable": True,
    "Size (Pixels)": 2,
    "Size (m)": 0.02,
    "Style": "Points",
    "Topic": get_topic(agent_name, "free_paths"),
    "Use Fixed Frame": True,
    "Use rainbow": True,
    "Value": True,
  })
  return rviz_config


def _add_point_cloud(agent_name: str, rviz_config: dict, name: str, topic_name: str, alpha: float) -> dict:
  rviz_config["Visualization Manager"]["Displays"].insert(0, {
    "Alpha": alpha,
    "Autocompute Intensity Bounds": True,
    "Autocompute Value Bounds": {
      "Max Value": 10,
      "Min Value": -10,
      "Value": True,
    },
    "Axis": "Z",
    "Channel Name": "intensity",
    "Class": "rviz_default_plugins/PointCloud2",
    "Color": ColorManager.get_color(),
    "Color Transformer": "FlatColor",
    "Decay Time": 0,
    "Enabled": True,
    "Invert Rainbow": False,
    "Max Color": "255; 255; 255",
    "Max Intensity": 4.0,
    "Min Color": "0; 0; 0",
    "Min Intensity": 0,
    "Name": f"{agent_name} {name}",
    "Position Transformer": "XYZ",
    "Selectable": True,
    "Size (Pixels)": 3,
    "Size (m)": 0.01,
    "Style": "Points",
    "Topic": get_topic(agent_name, topic_name),
    "Use Fixed Frame": True,
    "Use rainbow": True,
    "Value": True,
  })
  return rviz_config


def add_trajectory(agent_name: str, rviz_config: dict) -> dict:
  return _add_point_cloud(agent_name, rviz_config, "Trajectory", "trajectory", 0.5)


def add_explored_area(agent_name: str, rviz_config: dict) -> dict:
  return _add_point_cloud(agent_name, rviz_config, "Explored Area", "explored_areas", 0.2)


class ColorManager:
  colors = [
    "255; 0; 0",  # red
    "0; 255; 0",  # green
    "0; 0; 255",  # blue
    "255; 255; 0",  # yellow
    "255; 0; 255",  # magenta
    "0; 255; 255",  # cyan
  ]
  color_index = 0

  @classmethod
  def get_color(cls) -> str:
    return cls.colors[cls.color_index]

  @classmethod
  def tick(cls) -> None:
    cls.color_index = (cls.color_index + 1) % len(cls.colors)


def get_topic(agent_name: str, topic_name: str) -> dict:
  return {
    "Depth": 5,
    "Durability Policy": "Volatile",
    "Filter size": 10,
    "History Policy": "Keep Last",
    "Reliability Policy": "Reliable",
    "Value": f"/{agent_name}/{topic_name}",
  }


if __name__ == "__main__":
  main()
