"""Custom quadruped constants."""

from pathlib import Path

import mujoco
from mjlab.actuator import XmlPositionActuatorCfg
from mjlab.entity import EntityArticulationInfoCfg, EntityCfg
from mjlab.utils.os import update_assets
from mjlab.utils.spec_config import CollisionCfg

from src import SRC_PATH

MY_QUADRUPED_XML: Path = (
  SRC_PATH / "assets" / "robots" / "my_quadruped" / "mjcf" / "my_quadruped.xml"
)
assert MY_QUADRUPED_XML.exists()


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
    r"^(FL|FR|RL|RR)_hip_joint$": 0.0,
    r"^(FL|FR)_thigh_joint$": 0.6151,
    r"^(RL|RR)_thigh_joint$": 0.6519,
    r"^(FL|FR)_calf_joint$": -0.9065,
    r"^(RL|RR)_calf_joint$": -0.9709,
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
