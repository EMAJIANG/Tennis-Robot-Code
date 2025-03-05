# Copyright (c) 2021-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import numpy as np
from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import DynamicCuboid, VisualCuboid, DynamicSphere, GroundPlane

my_world = World(stage_units_in_meters=1.0)

# cube_1 = my_world.scene.add(
#     VisualCuboid(
#         prim_path="/new_cube_1",
#         name="visual_cube",
#         position=np.array([0, 0, 0.5]),
#         size=0.3,
#         color=np.array([255, 255, 255]),
#     )
# )

# cube_2 = my_world.scene.add(
#     DynamicCuboid(
#         prim_path="/new_cube_2",
#         name="cube_1",
#         position=np.array([0, 0, 1.0]),
#         scale=np.array([0.6, 0.5, 0.2]),
#         size=1.0,
#         color=np.array([255, 0, 0]),
#     )
# )

# cube_3 = my_world.scene.add(
#     DynamicCuboid(
#         prim_path="/new_cube_3",
#         name="cube_2",
#         position=np.array([0, 0, 3.0]),
#         scale=np.array([0.1, 0.1, 0.1]),
#         size=1.0,
#         color=np.array([0, 0, 255]),
#         linear_velocity=np.array([0, 0, 0.4]),
#     )
# )
# ground_plane = my_world.scene.add_default_ground_plane()
ground_plane = GroundPlane(prim_path="/World/GroundPlane", z_position=0)
Ground_material = PhysicsMaterial(
    prim_path="/World/Ground_Material",  # path to the material prim to create
    dynamic_friction=0.0,
    static_friction=0.0,
    restitution=1
)

ground_plane.apply_physics_material(Ground_material)

tennis_ball = DynamicSphere(
        prim_path=f"/World/tennis_ball_{1}",
        name=f"tennis_ball_{1}",
        position=np.array([1.0 , 1.0, 3.0]),
        radius=0.033,
        color=np.array([1.0, 0, 0]),
        mass=0.057
    )

material = PhysicsMaterial(
        prim_path=f"{tennis_ball.prim_path}/physicsMaterial",
        dynamic_friction=0.4,
        static_friction=1.1,
        restitution=1.0
    )
tennis_ball.apply_physics_material(material)
my_world.scene.add(tennis_ball)


for i in range(5):
    my_world.reset()
    for i in range(500):
        my_world.step(render=True)
        # print(cube_2.get_angular_velocity())
        # print(cube_2.get_world_pose())

simulation_app.close()
