from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import sys

import carb
import numpy as np
import math as math
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import add_reference_to_stage, get_stage_units
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.nucleus import get_assets_root_path
from omni.isaac.core.objects import DynamicSphere, GroundPlane
from omni.isaac.core.materials import PhysicsMaterial
import omni.kit.actions.core
from omni.isaac.core.prims import XFormPrimView, XFormPrim
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

Ground_plane = my_world.scene.add_default_ground_plane(restitution = 1)

Ground_material = PhysicsMaterial(
    prim_path="/World/Ground_Material",  # path to the material prim to create
    dynamic_friction=0.0,
    static_friction=0.0,
    restitution=1
)

Ground_plane.apply_physics_material(Ground_material)

TR_asset_path = "/home/ukygogo/isaacsim/Tennis-Robot-Code-main/TR_V4.usd"
add_reference_to_stage(usd_path=TR_asset_path, prim_path="/World/TR_V4")
TR_V4 = my_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

Court_asset_path = "/home/ukygogo/isaacsim/Tennis-Robot-Code-main/Court.usd"
add_reference_to_stage(usd_path=Court_asset_path, prim_path="/World/Court")
Court = my_world.scene.add(XFormPrim(prim_path="/World/Court" ,
                                           name="Court",
                                           position=np.array([-1.5,12,0])))#-1.5,12,0

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

initial_velocity = np.array([
    0.0,#x
    -10,#y
    0.0#z
])

initial_position = np.array([2.5, 12, 1])

def calculate_rebound_peak(initial_position, initial_velocity, restitution, gravity=9.81, Transition_Matrix = np.array([
[-1,  0,  0,  2],
[ 0, -1,  0,  2],
[ 0,  0,  1,  0],
[ 0,  0,  0,  1]])):

    v_x0 = initial_velocity[0]
    v_y0 = initial_velocity[1]
    v_z0 = initial_velocity[2]

    x0 = initial_position[0]
    y0 = initial_position[1]
    z0 = initial_position[2]

    t1 = (v_z0 + np.sqrt(v_z0**2 + 2*g*z0))/g
    v_z1 = (-v_z0 + g*t1)*restitution
    t_up = v_z1/g
    t_total = t1 + t_up


    # 计算总时间
    t_trajectory = np.linspace(0, t_total, 5000)

    # x, y方向的运动（匀速直线运动）
    x_trajectory = x0 + v_x0 * t_trajectory
    y_trajectory = y0 + v_y0 * t_trajectory

    # z方向的运动（分为下落和弹起）
    z_trajectory = np.zeros_like(t_trajectory)
    for i, t in enumerate(t_trajectory):
        if t <= t1:
            z_trajectory[i] = z0 + v_z0 * t - 0.5 * g * t**2  # 下落
        elif t > t1 and t <= t1 + t_up:
            z_trajectory[i] = 0 + (v_z1) * (t - t1) - 0.5 * g * (t - t1)**2  # 弹起

    x_peak = x_trajectory[-1]
    y_peak = y_trajectory[-1]
    z_peak = z_trajectory[-1]
        
    # print("target point:")
    # print(np.array([x_peak, y_peak, z_peak]))
    TR_Motion = np.dot(Transition_Matrix,np.array([x_peak, y_peak, z_peak, 1]).T)
    # print("TR location")
    # print(TR_Motion)
    return TR_Motion, t_total

# Function to move the robot to target position
def move_robot_to_target(robot,initial_position,initial_velocity,Trans):
    """
    Moves the robot to the target position.
    """
    
    joint_positions, t_total_temp = calculate_rebound_peak(initial_position,initial_velocity,1,9.81, Transition_Matrix=Trans)
    # import pdb; pdb.set_trace()
    robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=[joint_positions[1]+0.3, joint_positions[0]+1.1, joint_positions[2]-0.55, -30/180*2*math.pi],joint_velocities=[1.5,1.5,0.6,0])#'X_Pris', 'Z_Pris_H', 'Z_Pris_V', 'Racket_Pev', bias x+0.7272, z+0.17412, joint_velocities didnt work, should set controller mode to mixed
    )
    #print('The robot velocity is: ',robot.get_joint_velocities())
    # import pdb; pdb.set_trace()
    return np.array([joint_positions[1]+0.3, joint_positions[0]+1.1, joint_positions[2]-0.55, 0])

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
    return tennis_ball



def main ():
    
    for i in range(10):
        print(f"#########################Iteration {i + 1}: resetting world...#########################")
        my_world.reset()

        #generate random ball position and speed
        initial_velocity = np.array([
            0.0,#x
            -10,#+np.random.uniform(-2,2),#y
            0.0#z
        ])
        initial_position = np.array([
            2.5, 
            12,
            1
        ])

        #calculate the distanc between the robot and the target position
        target_pos_temp, t_total = calculate_rebound_peak(initial_position, initial_velocity, 1, 9.81, Trans)
        target_pos_temp = np.array([target_pos_temp[0]+1.1, target_pos_temp[1]+0.3, target_pos_temp[2]-0.55, 1])
        initial_position_temp = np.array([1,1,0,1])
        # print(np.dot(Trans,initial_position_temp.T))
        distance = initial_position_temp-target_pos_temp
        # print(distance)
        max_distance = np.max(np.abs(distance))
        sim_step = round(max_distance/1.5*60)
        print(f"sim_step: {sim_step}")
        print(f"final position: {target_pos_temp}")
        set_up_step = sim_step//3*2

        # check if the target position is out of range
        if target_pos_temp[0]>2 or target_pos_temp[0]<-2 or target_pos_temp[1]>2 or target_pos_temp[1]<-2 or target_pos_temp[2]>1.2 or target_pos_temp[2]<0.3:
            print("Target position out of working evolope")

        if round(t_total*60)<sim_step-set_up_step:
            print("Moving speed is not enough")

    # wait someseconds to create ball (for simualte ball prediction in real)
        # Move robot to target position
        
        for j in range(set_up_step):  # Simulate until hit (60 steps per second) 
            my_world.step(render=True)
            move_robot_to_target(TR_V4,initial_position,initial_velocity,Trans)

        # create tennis ball 
        tennis_ball = ball_create(i)

        # hit tennis ball
        for j in range(1,sim_step):
            my_world.step(render=True)
            target_position = move_robot_to_target(TR_V4,initial_position,initial_velocity,Trans)
            # TR_state = TR_V4.get_default_state()
            # ball_state = tennis_ball.get_current_dynamic_state()
            if j==sim_step - set_up_step: #sim_step - set_up_step means reduce the steps that already run in the setup
                ml.hit_tennisball_forehand(TR_V4,target_position)
                break
            my_world.step(render=True)

        for j in range(1,120):
            my_world.step(render=True)
            ml.hit_tennisball_forehand(TR_V4,target_position)
        
        # delete tennis ball
        print(f"Removing tennis_ball_{i}")
        my_world.scene.remove_object(f"tennis_ball_{i}")

    simulation_app.close()

if __name__== '__main__':
    main()