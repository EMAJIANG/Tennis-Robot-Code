from isaacsim import SimulationApp

# **MotionController 负责唯一初始化 SimulationApp**
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import rclpy
import queue
import numpy as np
import omni.usd
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from EnvInit import EnvSetup  # **导入 EnvInit**
from BodyControl import BodyControl
from RacketControl import RacketControl
from One_ball_motivation import OneBall
from rclpy.executors import MultiThreadedExecutor  # input Body and Racket in same executor 

class MotionController(Node):
    def __init__(self):
        super().__init__("MotionController")

        ### Create Subscriber for point and bool topic
        # create bool subscriber
        self.bool_subscription = self.create_subscription(
            Bool,
            'user_command_bool',
            self.bool_callback,
            10
        )
         # create point subscriber
        self.point_subscription = self.create_subscription(
            Point,
            'user_command_point',
            self.point_callback,
            10
        )
        # instant racket and body control(apply init in class)
        # env initialization
        self.env = EnvSetup()
        self.ros_world = self.env.get_ros_world()
        self.body_control = BodyControl(self.env)
        self.racket_control = RacketControl(self.env)
        self.one_ball = OneBall(self.env)
        self.action_queue = queue.Queue()

        # create publisher
        self.Body_Pub = self.create_publisher(Point,'BodyControl_position',10)
        self.Racket_Pub = self.create_publisher(Bool,'RacketControl_command',10)
        self.TR_status_Pub = self.create_publisher(Point,'TR_current_status',10)
        self.Ball_Pub = self.one_ball.publisher_
        



        self.has_ball = False
        self.get_logger().info("Motion Controller Initialized")

    def bool_callback(self, msg: Bool):
        self.get_logger().info(f"Received bool command: {msg.data}")
        # 将接收到的 Bool 消息转发给下层模块
        self.Racket_Pub.publish(msg)
        self.get_logger().info("Forwarded bool command to RacketControl.")
    
    def point_callback(self, msg: Point):
        self.get_logger().info(f"Received point command: x={msg.x}, y={msg.y}, z={msg.z}")
        # 将接收到的 Point 消息转发给下层模块
        self.Body_Pub.publish(msg)
        self.get_logger().info("Forwarded point command to BodyControl.")

    def TR_status_update(self):
        # 这里可以添加代码来更新 TR 的状态
        position_temp = self.env.TR_V4.get_joint_positions()
        if position_temp is not None:
            position_temp = np.dot(self.body_control.Trans, np.array([position_temp[0], position_temp[1], position_temp[2], 1]).T)
            position_temp = [position_temp[1] + 0.3, position_temp[0] + 1.1, position_temp[2]]
            msg = Point()
            msg.x = float(position_temp[0])
            msg.y = float(position_temp[1])
            msg.z = float(position_temp[2])
            self.TR_status_Pub.publish(msg)
            # self.get_logger().info(f"TR status updated: {self.body_control.joint_positions}")
        else:
            self.get_logger().warn("Failed to get TR joint positions.")

    
    def run_simulation(self):
        self.env.timeline.play()
        reset_needed = False

        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.body_control)
        executor.add_node(self.racket_control)
        executor.add_node(self.one_ball)

        while simulation_app.is_running():
            self.ros_world.step(render=True)
            executor.spin_once(timeout_sec=0.01)
            self.TR_status_update()
            if self.has_ball is False:
                self.one_ball.setup_scene()
                self.has_ball = True
            else:
                pass
        
            if self.body_control.pending_action is True:
                self.body_control.move_robot()

            if self.racket_control.pending_action_hit is True:
                self.racket_control.hit_racket()

            if self.racket_control.pending_action_recover is True:
                self.racket_control.hit_racket()

            if self.ros_world.is_stopped() and not reset_needed:
                reset_needed = True
            if self.ros_world.is_playing():
                if reset_needed:
                    self.ros_world.reset()
                    reset_needed = False

        self.env.timeline.stop()
        self.destroy_node()
        simulation_app.close()

if __name__ == "__main__":
    rclpy.init()
    motion_controller = MotionController()
    motion_controller.run_simulation()
