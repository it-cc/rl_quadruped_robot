"""Velocity environment configuration for the custom quadruped."""

from mjlab.envs import ManagerBasedRlEnvCfg
from mjlab.envs import mdp as envs_mdp
from mjlab.envs.mdp import dr
from mjlab.envs.mdp.actions import JointPositionActionCfg
from mjlab.managers import TerminationTermCfg
from mjlab.managers.event_manager import EventTermCfg
from mjlab.managers.observation_manager import ObservationTermCfg
from mjlab.managers.reward_manager import RewardTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.sensor import ContactMatch, ContactSensorCfg
from mjlab.tasks.velocity import mdp
from mjlab.utils.noise import UniformNoiseCfg as Unoise

from src.assets.robots import get_my_quadruped_robot_cfg
from src.tasks.velocity.latency import (
  apply_joint_feedback_delay,
  with_delayed_articulation,
)
from src.tasks.velocity.mdp import UniformVelocityCommandCfg
from src.tasks.velocity.velocity_env_cfg import make_velocity_env_cfg

FOOT_NAMES = ("FL", "FR", "RL", "RR")
CALF_GEOM_NAMES = tuple(f"{name}_calf_collision" for name in FOOT_NAMES)


def _my_quadruped_base_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create the custom quadruped velocity configuration."""
  cfg = make_velocity_env_cfg()
  cfg.sim.mujoco.timestep = 0.004
  cfg.decimation = 5
  cfg.scene.num_envs = 4096
  cfg.scene.entities = {
    "robot": with_delayed_articulation(get_my_quadruped_robot_cfg(), play=play),
  }

  feet_ground_cfg = ContactSensorCfg(
    name="feet_ground_contact",
    primary=ContactMatch(mode="geom", pattern=CALF_GEOM_NAMES, entity="robot"),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="netforce",
    num_slots=1,
    track_air_time=True,
  )
  nonfoot_ground_cfg = ContactSensorCfg(
    name="nonfoot_ground_touch",
    primary=ContactMatch(
      mode="geom",
      pattern=r".*",
      exclude=CALF_GEOM_NAMES,
      entity="robot",
    ),
    secondary=ContactMatch(mode="body", pattern="terrain"),
    fields=("found", "force"),
    reduce="none",
    num_slots=1,
    history_length=4,
  )
  cfg.scene.sensors = (cfg.scene.sensors or ()) + (
    feet_ground_cfg,
    nonfoot_ground_cfg,
  )

  actor_terms = cfg.observations["actor"].terms
  actor_terms.pop("base_lin_vel", None)
  actor_terms.pop("joint_pos", None)
  actor_terms.pop("joint_vel", None)
  actor_terms["base_accel"] = ObservationTermCfg(
    func=mdp.builtin_sensor,
    params={"sensor_name": "robot/imu_accel"},
    noise=Unoise(n_min=-0.5, n_max=0.5),
  )

  joint_pos_action = cfg.actions["joint_pos"]
  assert isinstance(joint_pos_action, JointPositionActionCfg)
  joint_pos_action.scale = 0.25
  apply_joint_feedback_delay(cfg, play=play)

  cfg.viewer.body_name = "trunk"
  cfg.viewer.distance = 1.5
  cfg.viewer.elevation = -10.0

  # 域随机化
  cfg.events["foot_friction"].params["asset_cfg"].geom_names = CALF_GEOM_NAMES
  cfg.events["foot_friction"].params["ranges"] = (0.2, 3.0)
  cfg.events["base_com"].params["asset_cfg"].body_names = ("trunk",)
  cfg.events["pd_gains"] = EventTermCfg(
    mode="startup",
    func=dr.pd_gains,
    params={
      "kp_range": (0.8, 1.2),
      "kd_range": (0.8, 1.2),
      "operation": "scale",
      # The XML position actuators are one actuator group with 12 targets.
      "asset_cfg": SceneEntityCfg("robot"),
    },
  )
  # 奖励设置
  cfg.rewards["pose"].params["std_standing"] = {
    r".*(FL|FR|RL|RR)_hip_joint.*": 0.05,
    r".*(FL|FR|RL|RR)_thigh_joint.*": 0.1,
    r".*(FL|FR|RL|RR)_calf_joint.*": 0.15,
  }
  cfg.rewards["pose"].params["std_walking"] = {
    r".*(FL|FR|RL|RR)_hip_joint.*": 0.15,
    r".*(FL|FR|RL|RR)_thigh_joint.*": 0.35,
    r".*(FL|FR|RL|RR)_calf_joint.*": 0.35,
  }
  cfg.rewards["pose"].params["std_running"] = {
    r".*(FL|FR|RL|RR)_hip_joint.*": 0.15,
    r".*(FL|FR|RL|RR)_thigh_joint.*": 0.5,
    r".*(FL|FR|RL|RR)_calf_joint.*": 0.5,
  }
  cfg.rewards["pose"].params["walking_threshold"] = 0.05
  cfg.rewards["pose"].params["running_threshold"] = 0.3

  cfg.rewards["track_linear_velocity"].weight = 4.0
  cfg.rewards["track_linear_velocity"].params["std"] = 0.15
  cfg.rewards["track_angular_velocity"].weight = 4.0
  cfg.rewards["track_angular_velocity"].params["std"] = 0.15

  cfg.rewards["body_ang_vel"].weight = -0.08
  cfg.rewards["angular_momentum"].weight = -0.03
  cfg.rewards["foot_gait"].params["period"] = 0.4
  cfg.rewards["foot_gait"].weight = 1.0
  cfg.rewards["foot_slip"].weight = -0.05

  cfg.rewards["foot_gait"].params["offset"] = [0.0, 0.5, 0.0, 0.5]
  cfg.rewards["body_orientation_l2"].params["asset_cfg"].body_names = ("trunk",)
  cfg.rewards["body_ang_vel"].params["asset_cfg"].body_names = ("trunk",)
  cfg.rewards["foot_clearance"].params["asset_cfg"].site_names = FOOT_NAMES
  cfg.rewards["foot_clearance"].params["target_height"] = 0.06
  cfg.rewards["foot_clearance"].weight = -0.5
  cfg.rewards["foot_slip"].params["asset_cfg"].site_names = FOOT_NAMES

  cfg.rewards["nonfoot_contact"] = RewardTermCfg(
    func=mdp.illegal_contact,
    weight=-3,
    params={"sensor_name": nonfoot_ground_cfg.name, "force_threshold": 0.5},
  )

  cfg.rewards["action_rate_l2"] = RewardTermCfg(
    func=mdp.action_rate_l2,
    weight=-0.2,
  )

  cfg.terminations["illegal_contact"] = TerminationTermCfg(
    func=mdp.illegal_contact,
    params={"sensor_name": nonfoot_ground_cfg.name, "force_threshold": 10.0},
  )

  if play:
    cfg.episode_length_s = int(1e9)
    cfg.observations["actor"].enable_corruption = False
    cfg.events.pop("push_robot", None)
    cfg.curriculum = {}
    cfg.events["randomize_terrain"] = EventTermCfg(
      func=envs_mdp.randomize_terrain,
      mode="reset",
      params={},
    )

    if (
      cfg.scene.terrain is not None and cfg.scene.terrain.terrain_generator is not None
    ):
      cfg.scene.terrain.terrain_generator.curriculum = False
      cfg.scene.terrain.terrain_generator.num_cols = 5
      cfg.scene.terrain.terrain_generator.num_rows = 5
      cfg.scene.terrain.terrain_generator.border_width = 10.0
    cfg.scene.num_envs = 1
  return cfg


def my_quadruped_flat_env_cfg(play: bool = False) -> ManagerBasedRlEnvCfg:
  """Create a flat-terrain custom quadruped velocity configuration."""
  cfg = _my_quadruped_base_env_cfg(play=play)
  
  cfg.sim.njmax = 300
  cfg.sim.mujoco.ccd_iterations = 50
  cfg.sim.contact_sensor_maxmatch = 64
  cfg.sim.nconmax = None

  assert cfg.scene.terrain is not None
  cfg.scene.terrain.terrain_type = "plane"
  cfg.scene.terrain.terrain_generator = None

  cfg.scene.sensors = tuple(
    s for s in (cfg.scene.sensors or ()) if s.name != "terrain_scan"
  )
  cfg.observations["actor"].terms.pop("height_scan", None)
  cfg.observations["critic"].terms.pop("height_scan", None)

  twist_cmd = cfg.commands["twist"]
  assert isinstance(twist_cmd, UniformVelocityCommandCfg)
  twist_cmd.heading_command = False
  twist_cmd.rel_heading_envs = 0.1
  twist_cmd.rel_standing_envs = 0.0
  twist_cmd.ranges.lin_vel_x = (-0.5, 0.6)
  twist_cmd.ranges.lin_vel_y = (-0.3, 0.3)
  twist_cmd.ranges.ang_vel_z = (-0.8, 0.8)
  twist_cmd.ranges.heading = None

  cfg.curriculum.pop("terrain_levels", None)
  cfg.curriculum.pop("command_vel", None)

  if play:
    twist_cmd.ranges.lin_vel_x = (-0.35, 0.45)
    twist_cmd.ranges.lin_vel_y = (-0.25, 0.25)
    twist_cmd.ranges.ang_vel_z = (-0.0, 0.0)
    cfg.scene.num_envs = 1
  return cfg
