# Copyright (c) 2020-2024, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

# ros2 topic pub /BodyControl_position geometry_msgs/msg/Point "{x: 4.0, y: 4.0, z: 1.0}"

# from isaacsim import SimulationApp

# simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# simulation_app.update()

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

import omni

from isaacsim.core.utils.extensions import enable_extension

# enable ROS2 bridge extension
enable_extension("isaacsim.ros2.bridge")

# simulation_app.update()

import time

# Note that this is not the system level rclpy, but one compiled for omniverse
import numpy as np
import math
import rclpy
import os
from rclpy.node import Node
from geometry_msgs.msg import Point
from isaacsim.core.prims import XFormPrim
from isaacsim.core.api.robots import Robot
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.utils.types import ArticulationAction


class BodyControl(Node):
    def __init__(self, env):
        super().__init__("BodyControl_subscriber")  
        self.ros_world = env.ros_world
        self.env = env
        self.Trans = np.array([
            [-1,  0,  0,  2],
            [ 0, -1,  0,  2],
            [ 0,  0,  1,  0],
            [ 0,  0,  0,  1]])
      
        # setup the ROS2 subscriber here
        self.Body_subscription = self.create_subscription(
            Point,
            'BodyControl_position',  # 主题名称
            self.position_callback,
            10  # 队列大小
        )
        self.TR_V4 = self.env.TR_V4
        self.Body_subscription  # 避免未被使用的警告
        self.joint_positions = np.array([0.0, 0.0, 0.0, -30 / 180 * 2 * math.pi]) 
        self.ros_world.reset()
        self.pending_position  = None
        self.pending_action = False
        self.joint_velocities=[1.5, 1.5, 0.6, 19]

    def position_callback(self, msg):
        if msg and hasattr(msg, "x") and hasattr(msg, "y") and hasattr(msg, "z"):
            self.joint_positions = np.array([msg.x, msg.y, msg.z])
            self.get_logger().info(f'Received position - X: {self.joint_positions[0]}, Y: {self.joint_positions[1]}, Z: {self.joint_positions[2]}')
            self.joint_positions = np.dot(self.Trans, np.array([self.joint_positions[0], self.joint_positions[1], self.joint_positions[2], 1]).T)
            # self.move_robot()
            self.pending_action = True
        else:
            self.get_logger().warn("Received invalid position message")

    def execute_pending_position(self):
        if self.pending_position is not None:
            # 更新 joint_positions
            self.joint_positions = self.pending_position
            self.move_robot()
            # 清除 pending_position
            self.pending_position = None
        else:
            self.get_logger().warn("Doesn't excute body")

    def move_robot(self):
        self.joint_positions_move_temp = [self.joint_positions[1] + 0.3, self.joint_positions[0] + 1.1, self.joint_positions[2] - 0.55, -30 / 180 * 2 * math.pi]
        self.action = ArticulationAction(
            joint_positions = self.joint_positions_move_temp,
            joint_velocities = self.joint_velocities
        )
        self.TR_V4.apply_action(self.action)
        self.actual_pos = np.dot(self.Trans, np.array([self.action.joint_positions[0], self.action.joint_positions[1], self.action.joint_positions[2], 1]).T)
        self.get_logger().info(f'Moving robot to new joint positions: {np.array([self.actual_pos[0]+ 0.3,self.actual_pos[1] + 1.1,self.actual_pos[2] + 0.55])}')
        self.pending_action = False

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

        # Cleanup
        self.timeline.stop()
        self.destroy_node()
        simulation_app.close()


if __name__ == "__main__":
    from isaacsim import SimulationApp
    simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})
    rclpy.init()
    BodyControl_subscriber = BodyControl()
    BodyControl_subscriber.run_simulation()
