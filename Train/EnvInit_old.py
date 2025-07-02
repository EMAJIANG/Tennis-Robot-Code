import omni
from isaacsim.core.api import World
from pxr import UsdGeom, PhysxSchema
import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np

import os
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from isaacsim.core.prims import XFormPrim
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
class EnvSetup:
    def __init__(self):        
        self.timeline = omni.timeline.get_timeline_interface() # simulation play or stop
        self.ros_world = World(stage_units_in_meters=1.0)   # create the world
        physics_dt = self.ros_world._physics_context.get_physics_dt()
        print(f"dt:{physics_dt}")
        ##### add asset court,robot,ground reference to the world
        current_path = os.getcwd()
        print("current_path:", current_path)
        Ground_asset_path = current_path +"/Tennis-Robot-Code-main/GroundPlane.usd"
        add_reference_to_stage(usd_path=Ground_asset_path, prim_path="/World/GroundPlane")

        TR_asset_path = current_path + "/Tennis-Robot-Code-main/TR_V4_ROS2.usd"
        add_reference_to_stage(usd_path=TR_asset_path, prim_path="/World/TR_V4")
        self.TR_V4 = self.ros_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

        Court_asset_path = current_path + "/Tennis-Robot-Code-main/Court.usd"
        add_reference_to_stage(usd_path=Court_asset_path, prim_path="/World/Court")
        Court = self.ros_world.scene.add(XFormPrim(prim_paths_expr="/World/Court", 
                                                name="Court",
                                                positions=np.array([[-1.5,12.0,0.0]])))#-1.5,12,0 
        print("EnvSetup initialized successfully.")
    
if __name__ == '__main__':
    env = EnvSetup()