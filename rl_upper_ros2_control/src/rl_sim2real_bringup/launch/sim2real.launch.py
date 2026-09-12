"""Bring up the ros2_control hardware and RL policy controller."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  OpaqueFunction,
  RegisterEventHandler,
)
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile


def _launch_setup(context):
  urdf_path = LaunchConfiguration("urdf")
  urdf = Path(urdf_path.perform(context)).read_text(encoding="utf-8")

  robot_description = {"robot_description": urdf}
  control_node = Node(
    package="controller_manager",
    executable="ros2_control_node",
    output="screen",
    parameters=[
      robot_description,
      ParameterFile(LaunchConfiguration("controllers"), allow_substs=True),
    ],
  )

  robot_state_publisher = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    output="screen",
    parameters=[robot_description],
  )

  controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
      "rl_sim2real_controller",
      "--controller-manager",
      "/controller_manager",
      "--controller-manager-timeout",
      "10",
    ],
    output="screen",
  )

  return [
    robot_state_publisher,
    control_node,
    RegisterEventHandler(
      OnProcessStart(target_action=control_node, on_start=[controller_spawner])
    ),
  ]


def generate_launch_description() -> LaunchDescription:
  robot_description_share = Path(
    get_package_share_directory("rl_sim2real_robot_description")
  )
  controller_share = Path(get_package_share_directory("rl_sim2real_controller"))
  default_urdf = robot_description_share / "urdf" / "my_quadruped.urdf"
  default_controllers = controller_share / "config" / "rl_sim2real_controllers.yaml"
  return LaunchDescription(
    [
      DeclareLaunchArgument("urdf", default_value=str(default_urdf)),
      DeclareLaunchArgument("controllers", default_value=str(default_controllers)),
      OpaqueFunction(function=_launch_setup),
    ]
  )
