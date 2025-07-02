import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from isaacsim.core.api.objects.sphere import DynamicSphere
from isaacsim.core.api.materials import PhysicsMaterial


class OneBall(Node):
    def __init__(self, env):
        super().__init__('one_ball_node')
        self.velocity_publisher = self.create_publisher(Twist, 'tennis_ball/velocity', 10)
        self.position_publisher = self.create_publisher(Pose, 'tennis_ball/pose', 10)
        self.timer = self.create_timer(1.0/120, self.publish_status)  # 每0.1秒发布一次球状态
        self.ros_world = env.ros_world
        self.tennis_ball = None

        # 初始球参数
        self.ini_position = np.array([2.5, 12.0, 1.0])

        self.ball_index = 0
        self.last_ball_name = None

    def setup_scene(self):
        ball_name = "tennis_ball"


        # 创建新的球体
        self.tennis_ball = DynamicSphere(
            prim_path=f"/World/{ball_name}",
            name=ball_name,
            position=self.ini_position,
            radius=0.033,
            color=np.array([1.0, 1.0, 0.0]),
            mass=0.057
        )

        material = PhysicsMaterial(
            prim_path=f"/World/{ball_name}/physicsMaterial",
            dynamic_friction=0.4,
            static_friction=0.8,
            restitution=0.9
        )
        self.tennis_ball.apply_physics_material(material)
        print(f"[OneBall] Created and launched.")

    def publish_status(self):
        if self.tennis_ball is not None:
            try:
                linear_velocity = self.tennis_ball.get_linear_velocity()
                angular_velocity = self.tennis_ball.get_angular_velocity()
                position, orientation = self.tennis_ball.get_world_pose()

                # 发布速度
                vel_msg = Twist()
                vel_msg.linear.x = float(linear_velocity[0])
                vel_msg.linear.y = float(linear_velocity[1])
                vel_msg.linear.z = float(linear_velocity[2])
                vel_msg.angular.x = float(angular_velocity[0])
                vel_msg.angular.y = float(angular_velocity[1])
                vel_msg.angular.z = float(angular_velocity[2])
                self.velocity_publisher.publish(vel_msg)

                # 发布位置
                pose_msg = Pose()
                pose_msg.position.x = float(position[0])
                pose_msg.position.y = float(position[1])
                pose_msg.position.z = float(position[2])
                pose_msg.orientation.x = float(orientation[0])
                pose_msg.orientation.y = float(orientation[1])
                pose_msg.orientation.z = float(orientation[2])
                pose_msg.orientation.w = float(orientation[3])
                self.position_publisher.publish(pose_msg)

            except Exception as e:
                self.get_logger().error(f"Error in publish_status: {e}")

    def run_simulation(self):
        self.setup_scene()
        for _ in range(20000):  # simulate ~3 seconds at 60 FPS
            self.ros_world.step(render=True)
            rclpy.spin_once(self, timeout_sec=0.01)

def main(args=None):
    rclpy.init(args=args)
    # 注意：实际使用时，env对象应该从外部传入
    # 下面是伪代码示范，不要直接运行
    # env = YourIsaacSimEnv()
    # one_ball = OneBall(env)
    # one_ball.run_simulation()
    # one_ball.destroy_node()
    # rclpy.shutdown()
    pass

if __name__ == '__main__':
    main()
