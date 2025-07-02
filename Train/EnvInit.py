import omni

from isaacsim.core.api import World
from pxr import UsdGeom, PhysxSchema
import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import omni.graph.core as og
print(og.get_node_type("isaacsim.ros2.bridge.ROS2SubscribeJointState"))
print(og.get_node_type("isaacsim.ros2.bridge.ROS2PublishJointState"))
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
        self.ros_world.set_simulation_dt(physics_dt=1.0 / 120.0, rendering_dt=0.0)
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

        # prim_path = "/World/TR_V4"
        # joint_states_topic = "/World/TR_V4/TR_joint_states"
        # joint_command_topic = "/World/TR_V4/TR_joint_commands"

        # keys = og.Controller.Keys

        # og.Controller.edit(
        #     {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        #     {
        #     og.Controller.Keys.CREATE_NODES: [
        #         ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
        #         ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
        #         ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
        #         ("ArticulationController", "omni.isaac.core_nodes.IsaacArticulationController"),
        #         ("ReadSimTime", "omni.isaac.core_nodes.IsaacReadSimulationTime"),
        #         ("PublishClock", "omni.isaac.ros2_bridge.ROS2PublishClock"),
        #         ("Context", "omni.isaac.ros2_bridge.ROS2Context"),
        #         ("ConstantTokenTarget", "omni.graph.nodes.ConstantToken"),
        #         ("ToTarget", "omni.graph.nodes.ToTarget"),
        #         ("ConstantStringJointStatesTopic", "omni.graph.nodes.ConstantString"),
        #         ("ConstantStringJointCommandTopic", "omni.graph.nodes.ConstantString"),
        #         ],
        #     og.Controller.Keys.CONNECT: [
        #         ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
        #         ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
        #         ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
        #         ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
        #         ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
        #         ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
        #         ("Context.outputs:context", "PublishJointState.inputs:context"),
        #         ("Context.outputs:context", "SubscribeJointState.inputs:context"),
        #         ("Context.outputs:context", "PublishClock.inputs:context"),
        #         ("ConstantTokenTarget.inputs:value", "ToTarget.inputs:value"),
        #         ("ToTarget.outputs:converted", "PublishJointState.inputs:targetPrim"),
        #         ("ToTarget.outputs:converted", "ArticulationController.inputs:targetPrim"),
        #         ("ConstantStringJointStatesTopic.inputs:value", "PublishJointState.inputs:topicName"),
        #         ("ConstantStringJointCommandTopic.inputs:value", "SubscribeJointState.inputs:topicName"),
        #         ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
        #         ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
        #         ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
        #         ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        #         ],
        #     og.Controller.Keys.SET_VALUES: [
        #         ("ConstantTokenTarget.inputs:value", prim_path),
        #         ("ConstantStringJointStatesTopic.inputs:value", joint_states_topic),
        #         ("ConstantStringJointCommandTopic.inputs:value", joint_command_topic),
        #         ],
        #     },
        # )
if __name__ == "__main__":
    env  = EnvSetup()
    