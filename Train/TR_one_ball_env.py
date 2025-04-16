import gymnasium as gym
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_msgs.msg import Empty
from queue import Queue
import time

from Train.EnvInit import EnvSetup # Import your environment setup

class IsaacOneBallEnv(gym.Env):
    def __init__(self):
        super().__init__()

        # Initialize ROS
        rclpy.init()
        self.node = Node("isaac_rl_env")

        # ROS Communication Setup
        self.cmd_pub_point = self.node.create_publisher(Point, 'user_command_point', 10)
        self.cmd_pub_bool = self.node.create_publisher(Bool, 'user_command_bool', 10)
        self.tr_status_sub = self.node.create_subscription(Point, 'TR_current_status', self._status_callback, 10)

        self.status_queue = Queue()

        # Isaac Sim Setup
        self.env = EnvSetup()
        self.ros_world = self.env.get_ros_world()
        self.timeline = self.env.timeline

        # Gym Spaces
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(3,), dtype=np.float32)  # Point(x, y, z)
        self.observation_space = gym.spaces.Box(low=-100.0, high=100.0, shape=(3,), dtype=np.float32)  # TR pos

        self.prev_position = np.zeros(3)
        self.target_position = np.array([0.5, 1.0, 0.5], dtype=np.float32)  # example target

    def _status_callback(self, msg):
        pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.status_queue.put(pos)

    def reset(self, *, seed=None, options=None):
        self.timeline.stop()
        self.ros_world.reset()
        time.sleep(0.2)
        self.timeline.play()

        # Clear status queue
        with self.status_queue.mutex:
            self.status_queue.queue.clear()

        # Initial action to stabilize the robot
        initial_point = Point(x=0.0, y=0.0, z=0.0)
        self.cmd_pub_point.publish(initial_point)

        # Wait for initial observation
        time.sleep(0.2)
        obs = self._get_latest_obs()
        self.prev_position = obs.copy()
        return obs, {}

    def step(self, action):
        # Map action to ROS Point message
        point_cmd = Point(x=float(action[0]), y=float(action[1]), z=float(action[2]))
        self.cmd_pub_point.publish(point_cmd)

        # Optional: trigger racket with bool
        if np.random.rand() > 0.5:
            self.cmd_pub_bool.publish(Bool(data=True))

        time.sleep(0.1)
        self.ros_world.step(render=True)

        obs = self._get_latest_obs()
        reward = self._compute_reward(self.prev_position, obs, action)
        self.prev_position = obs.copy()

        done = False  # Add logic if needed
        return obs, reward, done, False, {}

    def _compute_reward(self, prev_position, curr_position, action):
        # Example reward: closer to target, less motion, smaller action
        dist_to_target = np.linalg.norm(curr_position - self.target_position)
        movement_penalty = np.linalg.norm(curr_position - prev_position)
        action_penalty = np.linalg.norm(action)

        reward = -dist_to_target - 0.1 * movement_penalty - 0.05 * action_penalty
        return reward

    def _get_latest_obs(self):
        if not self.status_queue.empty():
            return self.status_queue.get()
        else:
            return self.prev_position

    def close(self):
        self.timeline.stop()
        self.node.destroy_node()
        rclpy.shutdown()
