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
initial_speed = -2.0  # 初速度，单位：米/秒
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

initial_position = np.array([2.5, 4, 1])

def calculate_rebound_peak(initial_position, initial_velocity, restitution, gravity=9.81):
    """
    Calculate the 3D position of the ball's highest point after rebounding once.

    Args:
        initial_position (np.array): Initial 3D position of the ball (x0, y0, z0).
        initial_velocity (np.array): Initial 3D velocity of the ball (vx, vy, vz).
        restitution (float): Coefficient of restitution (e.g., 0.8 for 80% energy retention).
        gravity (float): Gravitational acceleration (default is 9.81 m/s^2).

    Returns:
        np.array: 3D coordinates of the ball's highest point after the rebound (x, y, z).
    """
    # Decompose initial position and velocity
    x0, y0, z0 = initial_position
    print(initial_position)
    vx, vy, vz = initial_velocity
    print(initial_velocity)

    # Time to hit the ground (z = 0)
    t_to_ground = (vz + np.sqrt(vz**2 + 2 * gravity * z0)) / gravity
    x_ground = x0 + vx * t_to_ground
    y_ground = y0 + vy * t_to_ground
    z_ground = 0  # Ball hits the ground

    # Velocity after rebound
    vz_after = -restitution * vz

    # Time to reach the highest point after rebound
    t_to_peak = vz_after / gravity

    # Highest point after rebound
    x_peak = x_ground + vx * t_to_peak
    y_peak = y_ground + vy * t_to_peak
    z_peak = vz_after**2 / (2 * gravity)
    print("target point:")
    print(np.array([x_peak, y_peak, z_peak]))
    T = np.array([
    [-1,  0,  0,  2],
    [ 0, -1,  0,  2],
    [ 0,  0,  1,  0],
    [ 0,  0,  0,  1]
])
    print("TR location")
    print(np.dot(T,np.array([x_peak, y_peak, z_peak, 1]).T))
    return np.dot(T,np.array([x_peak, y_peak, z_peak, 1]).T)

# Function to move the robot to target position
def move_robot_to_target(robot):
    """
    Moves the robot to the target position.
    """
    joint_positions = calculate_rebound_peak(initial_position,initial_velocity,1,9.81)
    robot.get_articulation_controller().apply_action(
        ArticulationAction(np.array([joint_positions[0], joint_positions[1], joint_positions[2], 0]))
    )

for i in range(10):

    print(f"Iteration {i + 1}: resetting world...")
    my_world.reset()

    # 创建网球
    tennis_ball = DynamicSphere(
        prim_path=f"/World/tennis_ball_{i}",
        name=f"tennis_ball_{i}",
        position=initial_position,
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

    # Move robot to target position
    

    for j in range(int(10 * 60)):  # Simulate until hit (60 steps per second)
        move_robot_to_target(TR_V4)
        my_world.step(render=True)

    # 删除网球
    print(f"Removing tennis_ball_{i}")
    my_world.scene.remove_object(f"tennis_ball_{i}")

simulation_app.close()
