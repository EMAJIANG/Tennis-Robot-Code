# upgraded_TR_one_ball_env.py (Updated to detect and punish illegal hits)

import gymnasium as gym
import numpy as np
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Twist, Pose
from std_msgs.msg import Bool
from queue import Queue
import threading
import time
# from isaacsim.sensors.physics import ContactSensor

from Train.MotionControl import MotionController
# from Train.TR_one_ball_env import IsaacOneBallEnv
import omni.graph.core as og
print(og.get_node_type("isaacsim.ros2.bridge.ROS2SubscribeJointState"))
print(og.get_node_type("isaacsim.ros2.bridge.ROS2PublishJointState"))

class RewardAggregator:
    def __init__(self):
        self.reset()

    def reset(self):
        self.rewards = []
        self.max_neg_reward = float('-inf')
        self.exact_hit = False

    def add(self, nag_reward, is_exact):
        self.rewards.append(nag_reward)
        self.max_neg_reward = max(self.max_neg_reward, nag_reward)
        if is_exact:
            self.exact_hit = True

    def get_final_reward(self):
        if self.exact_hit:
            return 1.0
        return self.max_neg_reward

class IsaacOneBallEnv(gym.Env):
    def __init__(self):
        super().__init__()

        rclpy.init()
        self.node = Node("isaac_rl_env")
        timer_period = 1.0/50
        # self.pub_rate = self.node.create_timer(timer_period, self.timer_callback)

        self.motion_controller = MotionController()
        self.env = self.motion_controller.env
        self.ros_world = self.env.ros_world
        self.timeline = self.env.timeline

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.cmd_pub_point = self.node.create_publisher(Point, 'user_command_point', 10)
        self.cmd_pub_bool = self.node.create_publisher(Bool, 'user_command_bool', 10)
        self.tr_status_sub = self.node.create_subscription(Point, 'TR_current_status', self._status_callback, 10 ,callback_group=self.callback_group)
        self.ball_vel_sub = self.node.create_subscription(Twist, 'tennis_ball/velocity', self._ball_vel_callback, 10 ,callback_group=self.callback_group) 
        self.ball_pos_sub = self.node.create_subscription(Pose, 'tennis_ball/pose', self._ball_pos_callback, 10 ,callback_group=self.callback_group)

        self.executor = MultiThreadedExecutor(num_threads=12)
        self.executor.add_node(self.node)
        self.executor.add_node(self.motion_controller)
        self.executor.add_node(self.motion_controller.body_control)
        self.executor.add_node(self.motion_controller.racket_control)
        self.executor.add_node(self.motion_controller.one_ball)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True) #self.executor.spin
        self.spin_thread.start()



        self.status_queue = Queue()
        self.prev_distance = 0.0
        self.prev_position = np.zeros(3, dtype=np.float32)
        self.ball_velocity = np.zeros(6, dtype=np.float32)
        self.ball_velocity_before = np.zeros(6, dtype=np.float32)
        self.ball_position = np.zeros(3, dtype=np.float32)
        self.frame_skips = 30  # 可调节 frame 数量
        import random
        self.position_set_1 = [np.random.uniform(2, 3), 5.0, 1.0]
        self.position_set_2 = [np.random.uniform(2, 3), 6.0, 1.0]
        self.position_set_3 = [random.randrange(2, 3), 8.0, 1.0]
        self.position_set_4 = [np.random.uniform(2, 3), 10.0, 1.0]
        self.position_sets = [self.position_set_1, self.position_set_2, self.position_set_3, self.position_set_4]
        self.position_set_num = 0

        self.ball_velocity_set_1 = np.array([0.0, np.random.uniform(-6.0, -5.0), 3.2 ])
        self.ball_velocity_set_2 = np.array([0.0, np.random.uniform(-8.0, -7.0), 3.2 ])
        self.ball_velocity_set_3 = np.array([0.0, random.randrange(-10.0, -9.0), 3.2 ])
        self.ball_velocity_set_4 = np.array([0.0, np.random.uniform(-12.0, -1.0), 3.2 ])
        self.velocity_sets = [self.ball_velocity_set_1, self.ball_velocity_set_2, self.ball_velocity_set_3, self.ball_velocity_set_4]
        self.velocity_set_num = 0

        self.total_ball_num = 0
        self.successful_hits = 0
        self.total_ball_speed = 0
        self.total_steps = 0
        self.exact_num = 0
        self.exact_position_reward = 0.0

        self.world_move_x = None
        self.world_move_y = None
        self.world_move_z = None

        self.is_legal_hit = False
        self.prev_dx = 0.0
        self.prev_dy = 0.0
        self.prev_dz = 0.0

        self.Trans = np.array([
            [-1,  0,  0,  3.05],
            [ 0, -1,  0,  2.73],
            [ 0,  0,  1,  0.46],
            [ 0,  0,  0,  1]])
        self.Tw2r = np.linalg.inv(self.Trans)

        # self.action_space = gym.spaces.Box(
        #     low=np.array([-1, -1, -1, -1]),   # 每个关节的最小值
        #     high=np.array([1, 1, 1, 1]),     # 每个关节的最大值
        #     dtype=np.float32
        # )
        self.action_space = gym.spaces.Box( # no hitting
            low=np.array([-1, -1, -1]),   # 每个关节的最小值
            high=np.array([1, 1, 1]),     # 每个关节的最大值
            dtype=np.float32
        )
        self.observation_space = gym.spaces.Box(low= float('-inf'), high=float('inf'), shape=(3 + 3 + 6,), dtype=np.float32)
    def timer_callback(self):
        self.motion_controller.TR_status_update()
        if self.motion_controller.body_control.pending_action:
            self.motion_controller.body_control.move_robot()
        if self.motion_controller.racket_control.pending_action_recover:
            self.motion_controller.racket_control.recover_racket()

    def _thread_pub(self):
        while rclpy.ok:
            print("thread pub ok")
    def _status_callback(self, msg):
        pos = np.array([msg.x, msg.y, msg.z], dtype=np.float32)
        self.status_queue.put(pos)

    def _ball_vel_callback(self, msg):
        self.ball_velocity_before = self.ball_velocity.copy()
        self.ball_velocity = np.array([
            msg.linear.x, msg.linear.y, msg.linear.z,
            msg.angular.x, msg.angular.y, msg.angular.z
        ], dtype=np.float32)

    def _ball_pos_callback(self, msg):
        self.ball_position = np.array([
            msg.position.x, msg.position.y, msg.position.z
        ], dtype=np.float32)

    def _get_latest_obs(self):
        robot_obs = self.status_queue.get() if not self.status_queue.empty() else self.prev_position
        self.ball_velocity[9:12] = np.clip(self.ball_velocity[9:12],-100,100)
        return np.concatenate([robot_obs, self.ball_position, self.ball_velocity]).astype(np.float32)

    def reset(self, *, seed=None, options=None):
        self.timeline.pause()
        self.ros_world.step(render=False)
        self.want_to_hit = False
        if self.motion_controller.one_ball.tennis_ball is None:
            self.motion_controller.one_ball.setup_scene()

        self.ros_world.reset()
        self.ros_world.step(render=False)
        self.position_set_num = 2
        self.velocity_set_num = 2
        reset_position = self.position_sets[self.position_set_num]
        random_velocity = self.velocity_sets[self.velocity_set_num]
        self.motion_controller.one_ball.tennis_ball.set_world_pose(
            position=reset_position,
            orientation=np.array([0, 0, 0, 1])
        )
        self.motion_controller.one_ball.tennis_ball.set_linear_velocity(random_velocity)
        self.motion_controller.one_ball.tennis_ball.set_angular_velocity(np.array([0.0,0.0,0.0]))
        self.total_ball_num += 1
        # ball_prim _path = self.motion_controller.one_ball.tennis_ball.prim_path + "/contact_sensor"
        # self.contact_sensor = ContactSensor(
        #     prim_path=ball_prim_path,
        #     name="BallContactSensor",
        #     frequency=60,
        #     translation=np.array([0, 0, 0]),
        #     min_threshold=0.0,
        #     max_threshold=100000.0,
        #     radius=0.033
        # )
        # from pxr import PhysxSchema
        # physxAPI = PhysxSchema.PhysxRigidBodyAPI.Apply(self.motion_controller.one_ball.tennis_ball.prim)
        # physxAPI.CreateDisableGravityAttr(True)

        self.ros_world.step(render=False)
        
        self.timeline.play()
        self.start_sim_time = self.timeline.get_current_time()

        with self.status_queue.mutex:
            self.status_queue.queue.clear()
        self.motion_controller.racket_control.hit_racket()

        self.successful_hits = 0
        self.total_ball_speed = 0
        self.total_steps = 0

        obs, _ = self._wait_for_valid_obs()
        self.prev_position = obs[:3].copy()
        self.prev_distance =  np.linalg.norm(obs[:3]-obs[3:6])
        self.prev_dx = abs(obs[0] - obs[3])
        self.prev_dy = abs(obs[1] - obs[4])
        self.prev_dz = abs(obs[2] - obs[5])

        return obs, {}

    def _wait_for_valid_obs(self, timeout=5.0):
        start_time = time.time()
        while time.time() - start_time < timeout:
            obs = self._get_latest_obs()
            if not np.allclose(obs, 0.0):
                return obs, {}
            time.sleep(0.05)
        return obs, {}
        
    def step(self, action):
        self.reward_buffer = RewardAggregator()
        self.reward_buffer.reset()
        done = False
        max_steps = 150
        step_count = 0

        self.ros_world.step(render=True)
        # 控制动作
        self.world_move_x = action[0] * 2.0
        self.world_move_y = action[1] * 2.0
        self.world_move_z = (action[2] + 1.0) * 0.45 + 0.3
        self.cmd_pub_point.publish(Point(
                x=float(self.world_move_x),
                y=float(self.world_move_y),
                z=float(self.world_move_z)
                ))
        # 获取观测与奖励
        self.motion_controller.TR_status_update()
        if self.motion_controller.body_control.pending_action:
            self.motion_controller.body_control.move_robot()
        if self.motion_controller.racket_control.pending_action_recover:
            self.motion_controller.racket_control.recover_racket()
        obs = self._get_latest_obs()
        racket_pos = obs[:3]
        ball_pos = obs[3:6]
        ball_vel = obs[6:9]
        reward = self._compute_reward(racket_pos, ball_pos,ball_vel)
        is_exact = self.exact_position_reward>0

        # self.reward_buffer.add(reward, is_exact)

        # 终止条件
        speed = np.linalg.norm(self.ball_velocity[:3])
        step_count += 1
        too_long = step_count > max_steps
        out_of_bounds = (ball_pos[1] < 0) or (ball_pos[0] < 0) or (ball_pos[0] > 5)
        missing = (ball_pos[1]<racket_pos[1])
        too_slow = speed < 3
        done = too_long or out_of_bounds or too_slow
        suc_rate = self.exact_num/self.total_ball_num
        print(f"[Step]Reward:{reward} Exact times:{self.exact_num} Level num:{self.position_set_num} Successful Rate:{suc_rate}")
        # if suc_rate >= 1.1:
        #     if self.position_set_num ==2:
        #         pass
        #     self.position_set_num += 1
        #     self.velocity_set_num += 1
        #     self.exact_num = 0
        #     self.total_ball_num = 0
        #     print(f"Level up to {self.position_set_num}")
            
                
        return obs, float(reward), bool(done), False, {}

    def smooth_penalty(self, error, threshold):
        if error < threshold:
            return 1 - error / threshold
        else:
            return np.exp(threshold)-np.exp(error)  # 变成平滑指数惩罚

    def _compute_reward(self,curr_pos, ball_position, ball_vel):
        if ball_vel[2]<0:
            print(f"[INFO] Ball Position: {ball_position}, Racket Position: {curr_pos}")
            # 逐轴误差
            dx = abs(ball_position[0] - curr_pos[0])
            dy = abs(ball_position[1] - curr_pos[1])
            dz = abs(ball_position[2] - curr_pos[2])

            distance = np.linalg.norm(ball_position - curr_pos)  # 目标位置与当前球拍位置的距离

            distance_reward = 0.0
            # distance_reward = self.smooth_penalty(distance,0.05)  # 5cm 误差以内才有奖励
            distance_reward = self.prev_distance - distance
            x_reward = self.prev_dx - dx
            y_reward = self.prev_dy - dy
            z_reward = self.prev_dz - dz
            # 精度阈值（越小越鼓励精准）
            # x_reward = self.smooth_penalty(dx, 0.15)
            # y_reward = self.smooth_penalty(dy, 0.05)
            # z_reward = self.smooth_penalty(dz, 0.15)
            print(f"xreward:{x_reward}")
            print(f"yreward:{y_reward}")
            print(f"zreward:{z_reward}")
            print(f"distance_reward:{distance_reward}")
            total_reward = 0.0
            self.exact_position_reward = 0.0
            if distance_reward <= 0:
                total_reward = distance_reward #away punishment
            else:
                if x_reward > 0 and y_reward > 0 and z_reward > 0: #legal apporx
                    if ball_position[2] >1.3:
                        total_reward = distance_reward + x_reward + y_reward + z_reward
                    else:
                        total_reward = distance_reward + x_reward + y_reward + 10 * z_reward
                else:
                    if ball_position[2] >1.3:
                        total_reward = 10 * min(x_reward,y_reward,z_reward) #illegal approx
                    else:
                        total_reward = 20 * min(x_reward,y_reward,z_reward) #illegal approx

            # if dx<0.15 and dy<0.10 and dz<0.15: 
            #     self.exact_position_reward+=10.0
            #     total_reward += self.exact_position_reward #exact
            #     self.exact_num +=1
            if dx<0.10 and dy<0.05 and dz<0.10: 
                self.exact_position_reward+=30.0
                total_reward += self.exact_position_reward #exact
                self.exact_num +=1
            if dx<0.05 and dy<0.05 and dz<0.05:  
                self.exact_position_reward+=50.0
                total_reward += self.exact_position_reward #exact
                self.exact_num +=1
            # if self.want_to_hit:
            #     hit_reward = 10.0
            # if exact_position_reward>0 and self.want_to_hit:
            #     hit_reward = 20.0
            # if exact_position_reward>0 and not self.want_to_hit:
            #     hit_reward = -10.0
            # 最终总奖励
            # total_reward = 15 * distance_reward + 10 * self.exact_position_reward

            print(f"distance diff r:{distance_reward}")
            print(f"exact r:{self.exact_position_reward}")
            self.prev_distance = distance
            self.prev_dx = dx
            self.prev_dy = dy
            self.prev_dz = dz
            return total_reward
        else:
            return 0.0
            



    def close(self):
        self.timeline.stop()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()