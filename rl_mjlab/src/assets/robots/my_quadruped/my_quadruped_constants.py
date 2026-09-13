"""Custom quadruped constants."""

from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
from mjlab.actuator import XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

from src import SRC_PATH

MY_QUADRUPED_XML: Path = (
  SRC_PATH / "assets" / "robots" / "my_quadruped" / "mjcf" / "my_quadruped.xml"
)
MY_QUADRUPED_SCENE_XML: Path = MY_QUADRUPED_XML.parent / "scene.xml"
assert MY_QUADRUPED_XML.exists()
assert MY_QUADRUPED_SCENE_XML.exists()


def _get_default_joint_positions() -> dict[str, float]:
  """Read the training reset pose from the MuJoCo home keyframe."""
  scene_root = ET.parse(MY_QUADRUPED_SCENE_XML).getroot()
  key = scene_root.find("./keyframe/key[@name='home']")
  if key is None:
    raise ValueError("scene.xml must define keyframe 'home'")

  qpos_text = key.get("qpos")
  ctrl_text = key.get("ctrl")
  if qpos_text is None or ctrl_text is None:
    raise ValueError("scene.xml home keyframe must define qpos and ctrl")
  qpos = [float(value) for value in qpos_text.split()]
  ctrl = [float(value) for value in ctrl_text.split()]
  joint_names = (
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
  )
  joint_qpos = qpos[7:]
  if len(qpos) != 19 or len(ctrl) != len(joint_names):
    raise ValueError("scene.xml home keyframe must define 12 joint values")
  if joint_qpos != ctrl:
    raise ValueError("scene.xml home qpos and ctrl joint values must match")
  model_joint_names = [
    element.get("name")
    for element in ET.parse(MY_QUADRUPED_XML).getroot().iter("joint")
    if element.get("name") is not None
  ]
  if model_joint_names != ["float_base", *joint_names]:
    raise ValueError("my_quadruped.xml joint order does not match scene.xml")
  return dict(zip(joint_names, joint_qpos, strict=True))


def get_assets(meshdir: str) -> dict[str, bytes]:
  """Load mesh assets from the directory referenced by the MJCF."""
  assets: dict[str, bytes] = {}
  meshdir_clean = meshdir.rstrip("/")
  mesh_path = (MY_QUADRUPED_XML.parent / meshdir_clean).resolve()
  update_assets(assets, mesh_path, meshdir_clean)
  return assets


def get_spec() -> mujoco.MjSpec:
  """Load the custom quadruped MJCF and embed its mesh assets."""
  spec = mujoco.MjSpec.from_file(str(MY_QUADRUPED_XML))
  spec.assets = get_assets(spec.meshdir)
  return spec


MY_QUADRUPED_XML_ACTUATOR = XmlPositionActuatorCfg(
  target_names_expr=(".*",),
)


INIT_STATE = EntityCfg.InitialStateCfg(
  pos=(0.0, 0.0, 0.23),
  joint_pos={
    rf"^{name}$": value
    for name, value in _get_default_joint_positions().items()
  },
  joint_vel={r".*": 0.0},
)


CALF_FOOT_COLLISION = CollisionCfg(
  geom_names_expr=(r"^(FL|FR|RL|RR)_calf_collision$",),
  contype=0,
  conaffinity=1,
  condim=3,
  priority=1,
  friction=(0.6,),
  solimp=(0.9, 0.95, 0.023),
  disable_other_geoms=True,
)


MY_QUADRUPED_ARTICULATION = EntityArticulationInfoCfg(
  actuators=(MY_QUADRUPED_XML_ACTUATOR,),
  soft_joint_pos_limit_factor=0.9,
)


def get_my_quadruped_robot_cfg() -> EntityCfg:
  """Create a fresh entity configuration for the custom quadruped."""
  return EntityCfg(
    init_state=INIT_STATE,
    spec_fn=get_spec,
    articulation=MY_QUADRUPED_ARTICULATION,
    collisions=(CALF_FOOT_COLLISION,),
  )
