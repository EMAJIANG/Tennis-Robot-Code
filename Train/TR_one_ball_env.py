# upgraded_TR_one_ball_env.py (Updated to detect and punish illegal hits)
from omni.isaac.core.utils.extensions import enable_extension
enable_extension("isaacsim.ros2.bridge")
enable_extension("omni.isaac.core_nodes")
enable_extension("isaacsim.util.debug_draw")
import gymnasium as gym
import yaml
import os
import numpy as np
import math
import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, Twist, Pose
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from queue import Queue
import threading
import time
from Train.EnvInit import EnvSetup
from Train.One_ball_motivation import OneBall
import Train.Calibration

# from isaacsim.sensors.physics import ContactSensor

# from Train.MotionControl import MotionController

class IsaacOneBallEnv(gym.Env):
    def __init__(self):
        super().__init__()

        self.env = EnvSetup()
        rclpy.init()
        self.node = Node("isaac_rl_env")

        yaml_path = os.path.join(os.path.dirname(__file__), "../experiments/oneball_reward_stages.yaml")
        with open(yaml_path, 'r') as f:
            self.stage_cfg = yaml.safe_load(f)["stages"]
        self.current_stage = None
        self.stage_start = 0

        self.ros_world = self.env.ros_world
        self.one_ball = OneBall(self.env)
        self.timeline = self.env.timeline
        self.reset_mode = None

        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.action_pub = self.node.create_publisher(JointState, 'TR_Node/TR_joint_command', 10)
        self.status_sub = self.node.create_subscription(JointState, "TR_Node/TR_joint_states", self.states_sub_callback, 10)
        self.ball_vel_sub = self.node.create_subscription(Twist, 'tennis_ball/velocity', self._ball_vel_callback, 10 ,callback_group=self.callback_group) 
        self.ball_pos_sub = self.node.create_subscription(Pose, 'tennis_ball/pose', self._ball_pos_callback, 10 ,callback_group=self.callback_group)
        self.Ball_vel_Pub = self.one_ball.velocity_publisher
        self.Ball_pos_Pub = self.one_ball.position_publisher

        self.executor = MultiThreadedExecutor(num_threads=12)
        self.executor.add_node(self.node)

        self.executor.add_node(self.one_ball)
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True) #self.executor.spin
        self.spin_thread.start()
        self.joint_state = JointState()
        self.joint_state.name = ['X_Pris', 'Z_Pris_H', 'Z_Pris_V', 'Racket_Pev']
        self.joint_state.velocity = [1.5, 1.5, 1.5, 19.0]
        rate = self.node.create_rate(120)

        self.prev_distance = 0.0
        self.robot_obs = [0.0,0.0,0.0,1.0]
        self.racket_pos = None
        self.prev_position = np.zeros(3, dtype=np.float32)
        self.ball_velocity = np.zeros(6, dtype=np.float32)
        self.ball_velocity_before = np.zeros(6, dtype=np.float32)
        self.ball_position = np.zeros(3, dtype=np.float32)
        self.frame_skips = 600  # 可调节 frame 数量
        self.prev_ball_vel = np.zeros(6, dtype=np.float32)
        self.linar_distance = 1000.0

        self.step_count = 0
        self.total_ball_num = 0
        self.return_ball_num = 0
        self.return_num = 0
        self.total_ball_speed = 0
        self.is_hitted = False
        self.exact_position_reward = 0.0
        self.position_set_num = 0
        self.velocity_set_num = 0

        self.world_move_x = None
        self.world_move_y = None
        self.world_move_z = None
        self.racket_angle = None

        self.send_new_ball = False
        self.is_legal_hit = False
        self.prev_dx = 0.0
        self.prev_dy = 0.0
        self.prev_dz = 0.0

        self.action_space = gym.spaces.Box(
            low=np.array([-1, -1, -1, -1]),   # 每个关节的最小值
            high=np.array([1, 1, 1, 1]),     # 每个关节的最大值
            dtype=np.float32
        )

        self.observation_space = gym.spaces.Box(low= float('-inf'), high=float('inf'), shape=(3 + 3 + 6,), dtype=np.float32)
        self.timeline.play()
        self.node.get_logger().info("IsaacOneBallEnv initialized successfully.")
    def states_sub_callback(self, msg):
        #读取header
        # sec = msg.header.stamp.sec
        # nanosec = msg.header.stamp.nanosec
        # frame_id = msg.header.frame_id
        # print(f"Header: sec={sec}, nanosec={nanosec}, frame_id={frame_id}")

        # 读取关节名称
        #print("Joint Names:", msg.name)
        self.robot_obs[0] = msg.position[1]
        self.robot_obs[1] = msg.position[0]
        self.robot_obs[2] = msg.position[2]
        self.robot_obs[3] = math.degrees(msg.position[3])  # Racket angle
        self.robot_obs = Train.Calibration.calculate_racket_center(
            self.robot_obs[0], self.robot_obs[1], self.robot_obs[2], self.robot_obs[3])
        self.linar_distance = Train.Calibration.calculate_coordinates(
          msg.position[1], msg.position[0], msg.position[2], self.robot_obs[3],
            self.ball_position[0], self.ball_position[1], self.ball_position[2]
        )
        print(f"[Robot State] Position: {self.robot_obs[:3]}, Angle: {self.robot_obs[3]}")
        # print(self.robot_obs)
        # print(self.racket_pos)

    def training_rank(self,position_set_num, velocity_set_num):
        position_set_0 = [2.5, 5.2, 1.0]
        position_set_1 = [np.random.uniform(2, 3), 5.0, 1.0]
        position_set_2 = [np.random.uniform(2, 3), 6.0, 1.0]
        position_set_3 = [np.random.uniform(2, 3), 8.0, 1.0]
        position_set_4 = [np.random.uniform(2, 3), 10.0, 1.0]
        position_sets = [position_set_0, position_set_1, position_set_2, position_set_3, position_set_4]

        ball_velocity_set_0 = np.array([0.0, -5.7, 2.5])
        ball_velocity_set_1 = np.array([0.0, np.random.uniform(-6.0, -5.0), 3.2])
        ball_velocity_set_2 = np.array([0.0, np.random.uniform(-8.0, -7.0), 3.2])
        ball_velocity_set_3 = np.array([0.0, np.random.uniform(-10.0, -9.0), 3.2])
        ball_velocity_set_4 = np.array([0.0, np.random.uniform(-12.0, -1.0), 3.2])
        velocity_sets = [ball_velocity_set_0, ball_velocity_set_1, ball_velocity_set_2, ball_velocity_set_3, ball_velocity_set_4]

        return position_sets[position_set_num], velocity_sets[velocity_set_num]

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
        self.ball_velocity[9:12] = np.clip(self.ball_velocity[9:12],-100,100)
        return np.concatenate([self.robot_obs[:3], self.ball_position, self.ball_velocity]).astype(np.float32)
    
    def _update_stage(self):     
        for idx, stage in enumerate(self.stage_cfg):
            condition = stage["condition"]
            if eval(condition, globals(), {
                "ball_position": self.ball_position,
                "ball_vel": self.ball_velocity,
                "prev_ball_vel": self.prev_ball_vel,
                "step_count": self.step_count,
                "stage_start": self.stage_start,
                "np": np
            }):
                if self.current_stage != idx:
                    self.current_stage = idx
                    self.stage_start = self.step_count
                break

    def TR_action_command(self, action):
        self.world_move_x = action[0] * 2.0
        self.world_move_y = action[1] * 2.0
        self.world_move_z = (action[2] + 1.0)
        self.racket_angle = action[3]
        self.joint_state.position = [self.world_move_y, self.world_move_x, self.world_move_z, float(self.racket_angle)]
        # print(self.joint_state.position)
        self.action_pub.publish(self.joint_state)

    def reset(self, *, seed=None, options=None):
        print("[Reset]: in resetting...")
        self.timeline.stop()
        self.ros_world.step(render=False)
        if self.one_ball.tennis_ball is None:
            self.one_ball.setup_scene()

        if self.reset_mode is True:
            self.ros_world.reset()
        self.ros_world.step(render=False)
        self.position_set_num = 0
        self.velocity_set_num = 0
        reset_position, random_velocity = self.training_rank(self.position_set_num, self.velocity_set_num)
        self.one_ball.tennis_ball.set_world_pose(
            position=reset_position,
            orientation=np.array([1., 0., 0., 0.])
        )
        self.one_ball.tennis_ball.set_linear_velocity(random_velocity)
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

        if self.return_num >0:
            self.return_ball_num +=1
            self.return_num = 0
            
        self.timeline.play()
        self.step_count = 1
        self.one_ball.tennis_ball.set_angular_velocity(np.array([0.0, 0.0, 0.0]))
        self.total_ball_num += 1

        obs, _ = self._wait_for_valid_obs()
        self.prev_position = obs[:3].copy()
        self.prev_distance =  np.linalg.norm(obs[:3]-obs[3:6])
        self.prev_dx = abs(obs[0] - obs[3])
        self.prev_dy = abs(obs[1] - obs[4])
        self.prev_dz = abs(obs[2] - obs[5])
        self.prev_ball_vel = obs[6:9].copy()
        self.send_new_ball = False
        self.is_hitted = False
        self.reset_mode = False
        print("[Reset]: Resetting done")

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
        done = False
        max_steps = 150

        self.ros_world.step(render=True)
        # 控制动作
        self.TR_action_command(action)
        # 获取观测与奖励
        obs = self._get_latest_obs()
        racket_pos = obs[:3]
        ball_pos = obs[3:6]
        ball_vel = obs[6:9]
        reward = self._compute_reward(racket_pos, ball_pos,ball_vel, self.racket_angle)

        # 终止条件
        speed = np.linalg.norm(self.ball_velocity[:3])
        # too_long = self.step_count > max_steps
        out_of_bounds = (ball_pos[1] < 0) or (ball_pos[0] < 0) or (ball_pos[0] > 5)
        if out_of_bounds:
            self.step_count += 1
        time_step_reward = 0.0
        too_long = self.step_count >= 300 
        if too_long:
            self.reset_mode = True
        if self.send_new_ball is True:
            if self.return_num >0:
                self.return_ball_num +=1
                self.return_num = 0
            time_step_reward = 1/self.step_count
            print(f"[INFO] Send new ball, cost reward: {time_step_reward}")
        # missing = (ball_pos[1] - racket_pos[1]) < - 0.5
        too_slow = speed < 0.5
        done =  self.send_new_ball or too_long or too_slow
        if self.total_ball_num >0:
            reward += time_step_reward
            print(f"[Step]Reward:{reward} Level num:{self.position_set_num} return ball rate:{self.return_ball_num/self.total_ball_num}")# exact rate:{self.exact_num/self.total_ball_num}")
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

    def _compute_distance_reward(self, curr_pos, ball_position, back_to_center=False):
        total_distance_reward = 0.0
        self.exact_position_reward = 0.0
        distance_reward = 0.0
        if back_to_center is True:
            dx = abs(2.0 - curr_pos[0])
            dy = abs(2.0- curr_pos[1])
            dz = abs(0.6 - curr_pos[2])
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            distance_reward = self.prev_distance - distance
            x_reward = self.prev_dx - dx
            y_reward = self.prev_dy - dy
            z_reward = self.prev_dz - dz
            if distance_reward <= 0:
                total_distance_reward = -distance #away punishment
            else:
                if x_reward > 0 and y_reward > 0 and z_reward > 0: #legal apporx
                    total_distance_reward = distance_reward + x_reward + y_reward + z_reward
                    if dx < 0.3 and dy <  0.3 and dz < 0.2:
                        print('[INFO]could send a new ball')
                        self.send_new_ball = True
                        total_distance_reward += 1/distance #exact
                        pass
                else:
                    total_distance_reward = 10 * min(x_reward,y_reward,z_reward) #illegal approx
                    print(f"dx:{dx}, dy:{dy}, dz:{dz}")
            print(f"back_to_center xreward:{x_reward}")
            print(f"back_to_center yreward:{y_reward}")
            print(f"back_to_center zreward:{z_reward}")
            print(f"back_to_center distance_reward:{total_distance_reward}")
            self.prev_distance = distance
            self.prev_dx = dx
            self.prev_dy = dy
            self.prev_dz = dz

        if back_to_center is not True:
            distance = np.linalg.norm(ball_position - curr_pos)
            dx = abs(ball_position[0] - curr_pos[0])
            dy = abs(ball_position[1] - curr_pos[1])
            dz = abs(ball_position[2] - curr_pos[2])
            distance_reward = self.prev_distance - distance
            x_reward = self.prev_dx - dx
            y_reward = self.prev_dy - dy
            z_reward = self.prev_dz - dz
            print(f"dx:{dx}, dy:{dy}, dz:{dz}")
            print(f"xreward:{x_reward}")
            print(f"yreward:{y_reward}")
            print(f"zreward:{z_reward}")
            print(f"distance_reward:{distance_reward}")
            if distance_reward < 0:
                total_distance_reward = distance_reward #away punishment
            else:
                if x_reward > 0 and y_reward > 0 and z_reward > 0: #legal apporx
                    total_distance_reward = distance_reward + x_reward + y_reward + z_reward
                else:
                    total_distance_reward = 10 * min(x_reward,y_reward,z_reward) #illegal approx
            self.prev_distance = distance
            self.prev_dx = dx
            self.prev_dy = dy
            self.prev_dz = dz

        print(f"distance diff r:{distance_reward}")
        print(f"exact r:{self.exact_position_reward}")
        return total_distance_reward

    def _compute_reward(self, curr_pos, ball_position, ball_vel, racket_angle):
        self._update_stage()  # 自动阶段判断
        stage = self.stage_cfg[self.current_stage]["name"]
        reward = 0.0

        # --- Stage 1: Ball incoming ---
        if stage == "incoming":
            if (ball_position[1] > 0) and (ball_position[0] > 0):
                print(f"[STAGE: incoming] Ball incoming, approaching")
                reward += self._compute_distance_reward(curr_pos, ball_position, back_to_center=False)
            else:
                print(f"[STAGE: incoming] Ball incoming, missing or out of zone")
                reward += self._compute_distance_reward(curr_pos, ball_position, back_to_center=True)

            if self.is_hitted is True:
                print(f"[STAGE: incoming] Ball already bounced, returning to center")
                reward += self._compute_distance_reward(curr_pos, ball_position, back_to_center=True)

        # --- Stage 2: Miss or Recover ---
        elif stage == "miss_and_recover":
            print(f"[STAGE: miss_or_recover] Fallback or correction mode")
            reward += self._compute_distance_reward(curr_pos, ball_position, back_to_center=True)  # small penalty to encourage repositioning

        # --- Stage 3: Hitted ---
        elif stage == "hitted":
            print(f"[STAGE: hitted] Ball is just hit")
            if self.linar_distance > 0:
                reward += 1.0 / self.linar_distance  #

        # --- Stage 4: Forward motion after hit ---
        elif stage == "forward_and_recover":
            print(f"[STAGE: forward] Ball is flying forward")
            self.is_hitted = True
            reward += 10.0 * ball_vel[1] - 8.0 * ball_vel[0]
            reward += self._compute_distance_reward(curr_pos, ball_position, back_to_center=True)
            self.return_num += 1

        self.prev_ball_vel = ball_vel
        return reward

            

    def close(self):
        self.timeline.stop()
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown()