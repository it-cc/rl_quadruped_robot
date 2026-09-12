"""Velocity task for the custom quadruped."""

from mjlab.tasks.registry import register_mjlab_task

from src.tasks.velocity.rl import VelocityOnPolicyRunner

from .env_cfgs import my_quadruped_flat_env_cfg
from .rl_cfg import my_quadruped_ppo_runner_cfg

register_mjlab_task(
  task_id="MyQuadruped-Velocity-Flat",
  env_cfg=my_quadruped_flat_env_cfg(),
  play_env_cfg=my_quadruped_flat_env_cfg(play=True),
  rl_cfg=my_quadruped_ppo_runner_cfg(),
  runner_cls=VelocityOnPolicyRunner,
)
