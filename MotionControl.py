from isaacsim import SimulationApp

# **MotionController 负责唯一初始化 SimulationApp**
simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": False})

import rclpy
import queue
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from EnvInit import EnvSetup  # **导入 EnvInit**
from BodyControl import BodyControl
from RacketControl import RacketControl
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
        
        # create publisher
        self.Body_Pub = self.create_publisher(Point,'BodyControl_position',10)
        self.Racket_Pub = self.create_publisher(Bool,'RacketControl_command',10)
        
        # env initialization
        self.env = EnvSetup()
        self.ros_world = self.env.get_ros_world()

        # instant racket and body control(apply init in class)
        self.body_control = BodyControl(self.env)
        self.racket_control = RacketControl(self.env)

        self.action_queue = queue.Queue()

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
    
    def run_simulation(self):
        self.env.timeline.play()
        reset_needed = False

        executor = MultiThreadedExecutor()
        executor.add_node(self)
        executor.add_node(self.body_control)
        executor.add_node(self.racket_control)

        while simulation_app.is_running():
            self.ros_world.step(render=True)
            executor.spin_once(timeout_sec=0.01)

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
