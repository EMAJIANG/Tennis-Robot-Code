import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere
from isaacsim.core.api.materials import PhysicsMaterial
import omni.usd

class OneBall(Node):
    def __init__(self,env):
        super().__init__('one_ball_node')
        self.publisher_ = self.create_publisher(Twist, 'tennis_ball/velocity', 10)
        self.timer = self.create_timer(0.1, self.publish_velocity)
        self.ros_world = env.ros_world
        self.tennis_ball = None

    def setup_scene(self):

        # 创建网球
        self.tennis_ball = DynamicSphere(
            prim_path="/World/tennis_ball",
            name="tennis_ball",
            position=np.array([0.0, 0.0, 1.0]),
            radius=0.033,
            color=np.array([1.0, 1.0, 0.0]),
            mass=0.057
        )

        material = PhysicsMaterial(
            prim_path=f"{self.tennis_ball.prim_path}/physicsMaterial",
            dynamic_friction=0.4,
            static_friction=0.8,
            restitution=0.9
        )
        self.tennis_ball.apply_physics_material(material)
        self.ros_world.scene.add(self.tennis_ball)

        # 设置初始速度
        initial_velocity = np.array([2.0, 5.0, 3.0])
        self.tennis_ball.set_linear_velocity(initial_velocity)

    def publish_velocity(self):
        if self.tennis_ball:
            stage = omni.usd.get_context().get_stage()
            ball_prim = stage.GetPrimAtPath(self.tennis_ball.prim_path)
            if ball_prim:
                linear_velocity = self.tennis_ball.get_linear_velocity()
                angular_velocity = self.tennis_ball.get_angular_velocity() #degrees/s
                angular_velocity = angular_velocity * np.pi/180 # convert to rad/s
                if linear_velocity is not None and angular_velocity is not None:
                    msg = Twist()
                    msg.linear.x = float(linear_velocity[0])
                    msg.linear.y = float(linear_velocity[1])
                    msg.linear.z = float(linear_velocity[2])
                    msg.angular.x = float(angular_velocity[0])
                    msg.angular.y = float(angular_velocity[1])
                    msg.angular.z = float(angular_velocity[2])
                    self.publisher_.publish(msg)


    def run_simulation(self):
        self.setup_scene()
        for i in range(20000):  # 模拟3秒钟，假设60 FPS
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    one_ball = OneBall()
    one_ball.run_simulation()
    one_ball.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
