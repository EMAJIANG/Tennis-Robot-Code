from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys
import os
import carb
import numpy as np
import math as math
from isaacsim.core.api import World
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.core.api.materials import PhysicsMaterial
from isaacsim.core.api.objects import DynamicSphere
import omni.kit.actions.core
from isaacsim.core.prims import XFormPrim
from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, Usd, UsdLux, UsdRender, UsdUI

# import movement
import Movement_lib as ml

# Create a world
my_world = World(stage_units_in_meters=1.0)

action_registry = omni.kit.actions.core.get_action_registry()

# switches to camera lighting
action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_camera")
# switches to stage lighting
# action = action_registry.get_action("omni.kit.viewport.menubar.lighting", "set_lighting_mode_stage")
action.execute()

current_path = os.getcwd()
TR_asset_path = current_path + "/Tennis-Robot-Code-main/TR_V4.usd"
add_reference_to_stage(usd_path=TR_asset_path, prim_path="/World/TR_V4")
TR_V4 = my_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

Court_asset_path = current_path + "/Tennis-Robot-Code-main/Court.usd"
add_reference_to_stage(usd_path=Court_asset_path, prim_path="/World/Court")
Court = my_world.scene.add(XFormPrim(prim_paths_expr="/World/Court", 
                                           name="Court",
                                           positions=np.array([[-1.5,12.0,0.0]])))#-1.5,12,0

# Define tennis ball parameters
radius = 0.033  # Tennis ball radius, in meters (standard tennis ball diameter is 6.7 cm)
mass = 0.057  # Tennis ball mass, in kg
initial_speed = -2.0  # Initial speed, in meters per second
launch_angle_deg = -45.0  # Launch angle, in degrees
g = 9.81  # Gravity in m/s^2

# Transition Matrix
Trans = np.array([
    [-1,  0,  0,  2],
    [ 0, -1,  0,  2],
    [ 0,  0,  1,  0],
    [ 0,  0,  0,  1]])

# Convert angle to radians
launch_angle_rad = np.deg2rad(launch_angle_deg)

def calculate_rebound_peak(initial_position, initial_velocity, restitution, g=9.81, Transition_Matrix=Trans):
    v_x0 = initial_velocity[0]
    v_y0 = initial_velocity[1]
    v_z0 = initial_velocity[2]

    x0 = initial_position[0]
    y0 = initial_position[1]
    z0 = initial_position[2]

    t1 = (v_z0 + np.sqrt(v_z0**2 + 2*g*z0)) / g
    v_z1 = (-v_z0 + g*t1) * restitution
    t_up = v_z1 / g
    t_total = t1 + t_up

    # Calculate total time
    t_trajectory = np.linspace(0, t_total, 5000)

    # x, y motion (uniform straight motion)
    x_trajectory = x0 + v_x0 * t_trajectory
    y_trajectory = y0 + v_y0 * t_trajectory

    # z motion (split into fall and bounce)
    z_trajectory = np.zeros_like(t_trajectory)
    for i, t in enumerate(t_trajectory):
        if t <= t1:
            z_trajectory[i] = z0 + v_z0 * t - 0.5 * g * t**2  # Falling
        elif t > t1 and t <= t1 + t_up:
            z_trajectory[i] = 0 + v_z1 * (t - t1) - 0.5 * g * (t - t1)**2  # Bouncing

    x_peak = x_trajectory[-1]
    y_peak = y_trajectory[-1]
    z_peak = z_trajectory[-1]
    
    TR_Motion = np.dot(Transition_Matrix, np.array([x_peak, y_peak, z_peak, 1]).T)
    return TR_Motion, t_total

# Function to move the robot to target position
def move_robot_to_target(robot, initial_position, initial_velocity, Trans):
    joint_positions, t_total_temp = calculate_rebound_peak(initial_position, initial_velocity, 1, 9.81, Transition_Matrix=Trans)
    action = ArticulationAction(joint_positions=[joint_positions[1] + 0.3, joint_positions[0] + 1.1, joint_positions[2] - 0.55, -30 / 180 * 2 * math.pi], joint_velocities=[1.5, 1.5, 0.6, 0])
    robot.apply_action(action)
    # robot.ArticulationController.apply_action(
    #     ArticulationAction(joint_positions=[joint_positions[1] + 0.3, joint_positions[0] + 1.1, joint_positions[2] - 0.55, -30 / 180 * 2 * math.pi], joint_velocities=[1.5, 1.5, 0.6, 0])
    # )
    return np.array([joint_positions[1] + 0.3, joint_positions[0] + 1.1, joint_positions[2] - 0.55, 0])

# def add_tennis_ball_contact_sensor(my_world, i):
#     ball_contact_sensor = my_world.scene.add(
#             ContactSensor(
#                 prim_path=f"/World/tennis_ball_{i}" + "/contact_sensor",
#                 name="tennis_ball_contact_sensor_{}".format(i),
#                 min_threshold=0,
#                 max_threshold=10000000,
#                 radius=0.1,
#             )
#         )
#     return ball_contact_sensor

def ball_create(my_world, iteration, initial_position, initial_velocity): 
    i = iteration
    tennis_ball = DynamicSphere(
        prim_path=f"/World/tennis_ball_{i}",
        name=f"tennis_ball_{i}",
        position=initial_position,
        radius=radius,
        color=np.array([1.0, 0, 0]),
        mass=mass
    )

    material = PhysicsMaterial(
        prim_path=f"{tennis_ball.prim_path}/physicsMaterial",
        dynamic_friction=0.4,
        static_friction=1.1,
        restitution=1.0
    )
    tennis_ball.apply_physics_material(material)
    my_world.scene.add(tennis_ball)
    # ball_sensor = add_tennis_ball_contact_sensor(my_world, i)
    tennis_ball.set_linear_velocity(initial_velocity)
    return tennis_ball#, ball_sensor

Ground_plane = my_world.scene.add_default_ground_plane()

Ground_material = PhysicsMaterial(
    prim_path="/World/Ground_Material",  # path to the material prim to create
    dynamic_friction=0.0,
    static_friction=0.0,
    restitution=1
)

Ground_plane.apply_physics_material(Ground_material)

def main():
    for i in range(100000):
        print(f"#########################Iteration {i + 1}: resetting world...#########################")
        if i == 0:
            my_world.reset()
            TR_initial_position_temp = np.array([1.0, 1.0, 0.0, 1.0])
        else:
            TR_initial_position_temp = np.array([0.0, 0.0, 0.0, 1.0])

        # generate random ball position and speed
        initial_velocity = np.array([0.0, -10.0, 0.0])
        
        initial_position = np.array([2.5 , 12.0, 1.0])#+ np.random.uniform(-1, 1)

        target_pos_temp, t_total = calculate_rebound_peak(initial_position, initial_velocity, 1, 9.81, Trans)
        target_pos_temp = np.array([target_pos_temp[0] + 1.1, target_pos_temp[1] + 0.3, target_pos_temp[2] - 0.55, 1])
        distance = TR_initial_position_temp - target_pos_temp
        max_distance = np.max(np.abs(distance))
        sim_step = round(max_distance / 1.5 * 60)
        set_up_step = sim_step // 3 * 2

        if target_pos_temp[0] > 2 or target_pos_temp[0] < -2 or target_pos_temp[1] > 2 or target_pos_temp[1] < -2 or target_pos_temp[2] > 1.2 or target_pos_temp[2] < 0.3:
            print("Target position out of working envelope")

        if round(t_total * 60) < sim_step - set_up_step:
            print("Moving speed is not enough")

        for j in range(set_up_step):  # Simulate until hit (60 steps per second) 
            my_world.step(render=True)
            move_robot_to_target(TR_V4, initial_position, initial_velocity, Trans)

        ball_create(my_world, i, initial_position, initial_velocity)

        for j in range(1, sim_step):
            my_world.step(render=True)
            move_robot_to_target(TR_V4, initial_position, initial_velocity, Trans)
            if j == sim_step - set_up_step:
                ml.hit_tennisball_forehand(TR_V4, target_pos_temp)
                break
            my_world.step(render=True)

        for j in range(1, 120):
            my_world.step(render=True)
            ml.hit_tennisball_forehand(TR_V4, target_pos_temp)

        for j in range(1, 120):
            ml.reset(TR_V4)
            my_world.step(render=True)

        print(f"Removing tennis_ball_{i}")
        my_world.scene.remove_object(f"tennis_ball_{i}",True)

    simulation_app.close()

if __name__ == '__main__':
    main()
