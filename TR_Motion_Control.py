from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = "/home/lai-jiang/.local/share/ov/pkg/isaac-sim-4.2.0/TennisRobot/TR_V4.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/TR_V4")
TR_V4 = my_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

for i in range(10):
    print("resetting...")
    my_world.reset()
    TR_V4.set_joint_positions(np.array([0,0,0,0]))
    for j in range(500):
        my_world.step(render=True)
        if j == 100:
            TR_V4.get_articulation_controller().apply_action(
                ArticulationAction(joint_positions=np.array([np.random.uniform(-2, 2), np.random.uniform(-2, 2), np.random.uniform(-2, 2), np.random.uniform(-90, 90)]))
            )
        if j == 400:
            print("TR's joint positions are: ", TR_V4.get_joint_positions())
simulation_app.close()
