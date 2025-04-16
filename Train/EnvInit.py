import omni

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import math
import rclpy
import os
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from isaacsim.core.prims import XFormPrim
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction

class EnvSetup:
    def __init__(self):        
        self.timeline = omni.timeline.get_timeline_interface() # simulation play or stop
        self.ros_world = World(stage_units_in_meters=1.0)   # create the world

        ##### add asset court,robot,ground reference to the world
        current_path = os.getcwd()
        Ground_asset_path = current_path +"/GroundPlane.usd"
        add_reference_to_stage(usd_path=Ground_asset_path, prim_path="/World/GroundPlane")

        TR_asset_path = current_path + "/TR_V4.usd"
        add_reference_to_stage(usd_path=TR_asset_path, prim_path="/World/TR_V4")
        self.TR_V4 = self.ros_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

        Court_asset_path = current_path + "/Court.usd"
        add_reference_to_stage(usd_path=Court_asset_path, prim_path="/World/Court")
        Court = self.ros_world.scene.add(XFormPrim(prim_paths_expr="/World/Court", 
                                                name="Court",
                                                positions=np.array([[-1.5,12.0,0.0]])))#-1.5,12,0 
        
    def get_ros_world(self):
        return self.ros_world
    
