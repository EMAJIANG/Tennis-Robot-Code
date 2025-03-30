# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# from isaacsim import SimulationApp

# simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import omni

from isaacsim.core.api import World
from isaacsim.core.api.objects import VisualCuboid
from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# simulation_app.update()

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
from std_msgs.msg import Bool



class RacketControl(Node):
    def __init__(self, env):  # cannot use ros_world cuz TR_V4 is not in the ros_world
        super().__init__("RacketControl_subscriber")  
        self.ros_world = env.ros_world
        self.env = env # include TR_V4
        # setup the ROS2 subscriber
        self.Racket_subscription = self.create_subscription(
            Bool,  # whether hit 
            'RacketControl_command',  # customize topic name 
            self.angle_callback,
            10  # 队列大小
        )
        self.TR_V4 = self.env.TR_V4
        self.Racket_subscription  # 避免未被使用的警告
        self.joint_velocity=[19]
        self.joint_indices =[3]
        self.ros_world.reset()
        self.pending_action_recover = False
        self.pending_action_hit = False
        self.pending_action  = None

    def angle_callback(self, msg:Bool):
        if msg.data:
            self.get_logger().info('Recieved msg hit the racket')
            # self.hit_racket()
            self.pending_action_hit = True # record the require (pending)
        else:
            self.get_logger().info(f'Recieved msg recover the racket')
            # self.recover_racket()
            self.pending_action_recover = True # record the require (pending)

    def execute_pending_action(self):
        if hasattr(self, 'pending_action'):
            if self.pending_action == "hit":
                self.hit_racket()
        elif self.pending_action == "recover":
            self.recover_racket()
        self.pending_action = None  # 执行完毕后清除

    def hit_racket(self):
        # import pdb; pdb.set_trace()
        self.hit_position = [0 / 180 * 2 * math.pi]
        self.action = ArticulationAction(
            joint_indices = self.joint_indices,
            joint_positions = self.hit_position,
            joint_velocities = self.joint_velocity
        )
        self.TR_V4.apply_action(self.action)
        
        self.get_logger().info('Recieved command: hit the ball')
        self.pending_action_hit = False

    def recover_racket(self):
        self.recover_position = [-30 / 180 * 2 * math.pi]
        self.action = ArticulationAction(
            joint_indices = self.joint_indices,
            joint_positions = self.recover_position,
            joint_velocities = self.joint_velocity
        )
        self.TR_V4.apply_action(self.action)
        self.get_logger().info('Recieved command: recover the racket')
        self.pending_action_hit = False

    # def run_simulation(self):
    #     self.timeline.play()
    #     reset_needed = False
    #     while simulation_app.is_running():
    #         self.ros_world.step(render=True)
    #         rclpy.spin_once(self, timeout_sec=0.0)
    #         if self.ros_world.is_stopped() and not reset_needed:
    #             reset_needed = True
    #         if self.ros_world.is_playing():
    #             if reset_needed:
    #                 self.ros_world.reset()
    #                 reset_needed = False

        # Cleanup
        # self.timeline.stop()
        # self.destroy_node()


if __name__ == "__main__":
    rclpy.init()
    RacketControl_subscriber = RacketControl()
    RacketControl_subscriber.run_simulation()
