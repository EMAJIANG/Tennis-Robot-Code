# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

from isaacsim import SimulationApp

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import math
import rclpy
import EnvInit
import os
from rclpy.node import Node
from std_msgs.msg import Empty
from geometry_msgs.msg import Point
from isaacsim.core.prims import XFormPrim
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction


class BodyControl(Node):
    def __init__(self):
        super().__init__("BodyControl_subscriber")  
        self.Trans = np.array([
            [-1,  0,  0,  2],
            [ 0, -1,  0,  2],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  1]])
        self.timeline = omni.timeline.get_timeline_interface()
        self.ros_world = World(stage_units_in_meters=1.0)

        current_path = os.getcwd()
        Ground_asset_path = current_path +"/Tennis-Robot-Code-main/GroundPlane.usd"
        add_reference_to_stage(usd_path=Ground_asset_path, prim_path="/World/GroundPlane")

        TR_asset_path = current_path + "/Tennis-Robot-Code-main/TR_V4.usd"
        add_reference_to_stage(usd_path=TR_asset_path, prim_path="/World/TR_V4")
        self.TR_V4 = self.ros_world.scene.add(Robot(prim_path="/World/TR_V4", name="TR_V4"))

        Court_asset_path = current_path + "/Tennis-Robot-Code-main/Court.usd"
        add_reference_to_stage(usd_path=Court_asset_path, prim_path="/World/Court")
        Court = self.ros_world.scene.add(XFormPrim(prim_paths_expr="/World/Court", 
                                                name="Court",
                                                positions=np.array([[-1.5,12.0,0.0]])))#-1.5,12,0      
        # setup the ROS2 subscriber here
        self.subscription = self.create_subscription(
            Point,
            'BodyControl_position',  # 主题名称
            self.position_callback,
            10  # 队列大小
        )
        self.subscription  # 避免未被使用的警告
        self.joint_positions = np.array([0.0, 0.0, 0.0, -30 / 180 * 2 * math.pi])
        self.ros_world.reset()

    def position_callback(self, msg):
        self.joint_positions = np.array([msg.x, msg.y, msg.z])
        self.get_logger().info(f'Received position - X: {self.joint_positions[0]}, Y: {self.joint_positions[1]}, Z: {self.joint_positions[2]}')
        self.joint_positions = np.dot(self.Trans, np.array([self.joint_positions[0], self.joint_positions[1], self.joint_positions[2], 1]).T)

    def move_robot(self):
        action = ArticulationAction(
            joint_positions=[self.joint_positions[1] + 0.3, self.joint_positions[0] + 1.1, self.joint_positions[2] - 0.55, -30 / 180 * 2 * math.pi],
            joint_velocities=[1.5, 1.5, 0.6, 0]
        )
        self.TR_V4.apply_action(action)
        self.get_logger().info(f'Moving robot to new joint positions: {action.joint_positions}')

    def run_simulation(self):
        self.timeline.play()
        reset_needed = False
        while simulation_app.is_running():
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.0)
            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False
                # the actual setting the cube pose is done here
                self.move_robot()

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    rclpy.init()
    BodyControl_subscriber = BodyControl()
    BodyControl_subscriber.run_simulation()
