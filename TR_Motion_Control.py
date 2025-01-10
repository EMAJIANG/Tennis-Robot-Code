from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import argparse
import sys

import carb
import numpy as np
import math as math
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicSphere
from omni.isaac.core.materials import PhysicsMaterial

my_world = World(stage_units_in_meters=1.0)
my_world.scene.add_default_ground_plane()

asset_path = "/home/lai-jiang/.local/share/ov/pkg/isaac-sim-4.2.0/TennisRobot/TR_V4.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/TR_V4")
TR_V4 = my_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

# 定义网球的参数
radius = 0.033  # 网球半径，单位：米（标准网球直径为6.7cm）
mass = 0.057  # 网球质量，单位：千克
initial_speed = 1.0  # 初速度，单位：米/秒
launch_angle_deg = 45.0  # 发射角度，单位：度
g = 9.81  # gravity in m/s^2

# 将角度转换为弧度
launch_angle_rad = np.deg2rad(launch_angle_deg)

# 计算初速度的分量
initial_velocity = np.array([
    0.0,#x
    initial_speed * np.cos(launch_angle_rad),#y
    initial_speed * np.sin(launch_angle_rad)#z
])

# Function to predict tennis ball landing point
def predict_landing_point(position, velocity):
    """
    Predicts the landing point of the tennis ball using kinematics.
    """
    z0 = position[2]  # Initial height
    vz = velocity[2]  # Vertical velocity

    # Time to hit the ground (z = 0)
    time_to_ground = (vz + np.sqrt(vz**2 + 2 * g * z0)) / g

    # Final position in x and y after time_to_ground
    landing_position = position[:2] + velocity[:2] * time_to_ground
    return landing_position, time_to_ground

# Function to move the robot to target position
def move_robot_to_target(robot, target_position, time_to_hit):
    """
    Moves the robot to the target position.
    """
    joint_positions = np.array([
        target_position[0],  # Move to X
        target_position[1],  # Move to Y
        0.5,  # Set height for hitting
        0.0,  # Paddle angle
    ])
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=joint_positions)
    )

for i in range(10):

    print(f"Iteration {i + 1}: resetting world...")
    my_world.reset()

    # 创建网球
    tennis_ball = DynamicSphere(
        prim_path=f"/World/tennis_ball_{i}",
        name=f"tennis_ball_{i}",
        position=np.array([1, -2, 1]),
        radius=radius,
        color=np.array([1.0, 0.2, 0.2]),
        mass=mass
    )

    if not hasattr(tennis_ball, 'prim') or tennis_ball.prim is None:
        print(f"Error: Tennis ball {i} prim is not initialized!")
        continue

    material = PhysicsMaterial(
        prim_path=f"{tennis_ball.prim_path}/physicsMaterial",
        dynamic_friction=0.4,
        static_friction=1.1,
        restitution=1
    )
    tennis_ball.apply_physics_material(material)
    my_world.scene.add(tennis_ball)
    print(f"Tennis ball created: {tennis_ball.prim_path}")

    # Set initial velocity
    tennis_ball.set_linear_velocity(initial_velocity)

    # Predict landing point and time
    landing_position, time_to_hit = predict_landing_point(
        position=np.array([0, 0, 1]), velocity=initial_velocity
    )
    print(f"Predicted landing position: {landing_position}, time to hit: {time_to_hit}")

    # Move robot to target position
    move_robot_to_target(TR_V4, landing_position, time_to_hit)

    for j in range(int(30 * 60)):  # Simulate until hit (60 steps per second)
        my_world.step(render=True)

    # 删除网球
    print(f"Removing tennis_ball_{i}")
    my_world.scene.remove_object(f"tennis_ball_{i}")

simulation_app.close()
