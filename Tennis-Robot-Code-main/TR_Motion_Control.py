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
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicSphere, GroundPlane
from omni.isaac.core.materials import PhysicsMaterial
import omni.kit.actions.core
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade

my_world = World(stage_units_in_meters=1.0)

action_registry = omni.kit.actions.core.get_action_registry()

# switches to camera lighting
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
# switches to stage lighting
# action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_stage")
action.execute()

Ground_plane = my_world.scene.add_default_ground_plane(restitution = 1)

Ground_material = PhysicsMaterial(
    prim_path="/World/Ground_Material",  # path to the material prim to create
    dynamic_friction=0.0,
    static_friction=0.0,
    restitution=1
)

Ground_plane.apply_physics_material(Ground_material)

asset_path = "./TR_V4.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/TR_V4")
TR_V4 = my_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

# 定义网球的参数
radius = 0.033  # 网球半径，单位：米（标准网球直径为6.7cm）
mass = 0.057  # 网球质量，单位：千克
initial_speed = -2.0  # 初速度，单位：米/秒
launch_angle_deg = -45.0  # 发射角度，单位：度
g = 9.81  # gravity in m/s^2

#Transition Matrix
Trans = np.array([
[-1,  0,  0,  2],
[ 0, -1,  0,  2],
[ 0,  0,  1,  0],
[ 0,  0,  0,  1]])

# 将角度转换为弧度
launch_angle_rad = np.deg2rad(launch_angle_deg)

# 计算初速度的分量
# initial_velocity = np.array([
#     0.0,#x
#     initial_speed * np.cos(launch_angle_rad),#y
#     initial_speed * np.sin(launch_angle_rad)#z
# ])

initial_velocity = np.array([
    0.0,#x
    -2,#y
    -5#z
])

initial_position = np.array([2.5, 6, 1])

def calculate_rebound_peak(initial_position, initial_velocity, restitution, gravity=9.81, Transition_Matrix = np.array([
[-1,  0,  0,  2],
[ 0, -1,  0,  2],
[ 0,  0,  1,  0],
[ 0,  0,  0,  1]])):
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
    vx, vy, vz = initial_velocity

    # Time to hit the ground (z = 0)
    t_to_ground = (vz + np.sqrt(vz**2 + 2 * gravity * z0)) / gravity
    x_ground = x0 + vx * t_to_ground
    y_ground = y0 + vy * t_to_ground
    z_ground = 0  # Ball hits the ground

    # Velocity after rebound
    vz_after = -restitution * vz

    # Time to reach the highest point after rebound
    t_to_peak = vz_after / gravity

    # the time robot need to hit the ball
    time_move = t_to_ground + t_to_peak

    # Highest point after rebound
    x_peak = x_ground + vx * t_to_peak
    y_peak = y_ground + vy * t_to_peak
    z_peak = vz_after**2 / (2 * gravity)
    print("target point:")
    print(np.array([x_peak, y_peak, z_peak]))
    TR_Motion = np.dot(Transition_Matrix,np.array([y_peak, x_peak, z_peak, 1]).T)
    print("TR location")
    print(TR_Motion)
    return np.dot(Transition_Matrix,np.array([x_peak, y_peak, z_peak, 1]).T)

# Function to move the robot to target position
def move_robot_to_target(robot):
    """
    Moves the robot to the target position.
    """
    
    joint_positions = calculate_rebound_peak(initial_position,initial_velocity,1,9.81, Transition_Matrix=Trans)
    # import pdb; pdb.set_trace()
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=[joint_positions[1], joint_positions[0]+0.95, joint_positions[2]-0.17412, 0],joint_velocities=[10,5,5,0])#'X_Pris', 'Z_Pris_H', 'Z_Pris_V', 'Racket_Pev', bias x+0.7272, z+0.17412, joint_velocities didnt work, should set controller mode to mixed
    )
    print('The robot velocity is: ',robot.get_joint_velocities())
    # import pdb; pdb.set_trace()

def ball_create(iteration): 
    i = iteration
    tennis_ball = DynamicSphere(
        prim_path=f"/World/tennis_ball_{i}",
        name=f"tennis_ball_{i}",
        position=initial_position,
        radius=radius,
        color=np.array([1.0, 0.2, 0.2]),
        mass=mass
    )

    material = PhysicsMaterial(
        prim_path=f"{tennis_ball.prim_path}/physicsMaterial",
        dynamic_friction=0.4,
        static_friction=1.1,
        restitution=1
    )
    tennis_ball.apply_physics_material(material)
    my_world.scene.add(tennis_ball)
    print(f"Tennis ball created: {tennis_ball.prim_path}")
    tennis_ball.set_linear_velocity(initial_velocity)

def main ():
    for i in range(10):

        print(f"#########################Iteration {i + 1}: resetting world...#########################")
        my_world.reset()
    
       

    # wait someseconds to create ball (for simualte ball prediction in real)

        # Move robot to target position
        for j in range(int(10 * 30)):  # Simulate until hit (60 steps per second) 
            my_world.step(render=True)
            move_robot_to_target(TR_V4)

        # create tennis ball 
        ball_create(i)

        for j in range(10*30,int(10*60)):
            my_world.step(render=True)
            move_robot_to_target(TR_V4)
        # 删除网球
        print(f"Removing tennis_ball_{i}")
        my_world.scene.remove_object(f"tennis_ball_{i}")

    simulation_app.close()

if __name__== '__main__':
    main()

