#!/usr/bin/env python3
"""
Usage:
sudo python3 x_axis_ros2.py enp5s0f1

This script is the original controller extended with a ROS2 subscriber node:
Topic name: "TR_joint_command"
Message type: sensor_msgs.msg.JointState
Expect positions array: [x, y, z] in range [-2, 2] for x and y.

Mapping rules applied on incoming JointState.positions:
 - input_x in [-2, 2]  -> mapped_y in [-1, 1]   (linear)
 - input_y in [-2, 2]  -> mapped_x in [0, 1]    (shift: input_y = -2 -> mapped_x = 0)
 - input_z unchanged

Once a new message arrives, the newest mapped point will replace the trajectory queue
(i.e. queue length kept to 1) so the motors always execute the latest command.

Notes:
 - Requires ROS2 python package rclpy and sensor_msgs installed in the environment.
 - The ROS2 spinning runs in a background thread.
"""

import sys
import struct
import time
import collections
import threading

import pysoem

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# 定义CoE对象索引 (根据XML中的PDO条目更新)
X_CNT_2_DIG = 615000000
Z_CNT_2_DIG = 240000000
Y_CNT_2_DIG = 310000000
CONTROLWORD = 0x6040
STATUSWORD = 0x6041
MODE_OF_OPERATION = 0x6060
TARGET_POSITION = 0x607A
TARGET_VELOCITY = 0x60FF
MAX_TORQUE = 0x6072
ACTUAL_POSITION = 0x6064
DIGITAL_INPUTS = 0x60FD
ACTUAL_VELOCITY = 0x606C
PROFILE_VELOCITY = 0x6081
POSITION_RANGE_LIMIT = 0x607B
PROFILE_ACCELERATION = 0x6083
PROFILE_DECELERATION = 0x6084

profile_velocity = int(Y_CNT_2_DIG)
x_profile_acceleration = int(1 * Y_CNT_2_DIG)
x_profile_deceleration = int(1 * Y_CNT_2_DIG)
profile_acceleration = int(1.5 * Y_CNT_2_DIG)
profile_deceleration = int(1.5 * Y_CNT_2_DIG)

class TRJointSubscriber(Node):
    """ROS2 Node that subscribes to TR_joint_command and maps incoming JointState -> target point."""
    def __init__(self, owner):
        super().__init__('TR_joint_command_subscriber')
        self.owner = owner  # MinimalExample instance
        self.sub = self.create_subscription(
            JointState,
            'TR_joint_command',
            self.joint_callback,
            10
        )
        self.get_logger().info('TR_joint_command subscriber created')

    def joint_callback(self, msg: JointState):
        # 安全读取 positions（若长度不足，忽略）
        if not msg.position or len(msg.position) < 3:
            self.get_logger().warning('Received JointState with insufficient positions; ignoring')
            return

        in_x = float(msg.position[0])
        in_y = float(msg.position[1])
        in_z = float(msg.position[2])

        # 映射规则：
        # input_x [-2,2] -> mapped_y [-1,1]  (y_pos = 0.5 * in_x)
        # input_y [-2,2] -> mapped_x [0,1], with shift so in_y=-2 -> x=0 (x_pos = (in_y + 2) / 4)
        # z unchanged
        mapped_y = max(-1.0, min(1.0, 0.5 * in_x))
        mapped_x = max(0.0, min(1.0, (in_y + 2.0) / 4.0))
        mapped_z = in_z  # 保持不变

        # 将映射值转换为 counts （根据原脚本使用 X/Y/Z_CNT_2_DIG 定义）
        target_x_counts = int(mapped_x * X_CNT_2_DIG)
        target_y_counts = int(mapped_y * Y_CNT_2_DIG)
        target_z_counts = int(mapped_z * Z_CNT_2_DIG)

        # 更新主控制对象的轨迹队列：保持队列始终只有最新一个点
        with self.owner.trajectory_lock:
            self.owner.trajectory_points.clear()
            self.owner.trajectory_points.append({
                'x': target_x_counts,
                'y': target_y_counts,
                'z': target_z_counts
            })

        self.get_logger().info(
            f"Received JointState -> mapping to counts X:{target_x_counts}, Y:{target_y_counts}, Z:{target_z_counts} and replaced trajectory queue"
        )


class MinimalExample:

    BECKHOFF_VENDOR_ID = 0x0000066f
    ELMO_PRODUCT_CODE = 0x60380007

    def __init__(self, ifname):
        self._ifname = ifname
        self._master = pysoem.Master()
        SlaveSet = collections.namedtuple(
            'SlaveSet', 'slave_name product_code config_func')
        self._expected_slave_mapping = {0: SlaveSet('X1', self.ELMO_PRODUCT_CODE, self.elmo_setup),
                                        1: SlaveSet('X2', self.ELMO_PRODUCT_CODE, self.elmo_setup),
                                        2: SlaveSet('Y', self.ELMO_PRODUCT_CODE, self.elmo_setup),
                                        3: SlaveSet('Z', self.ELMO_PRODUCT_CODE, self.elmo_setup)
                                        }

        # 将轨迹队列设为实例属性，供 ROS 回调与控制循环共享
        self.trajectory_points = collections.deque()
        self.trajectory_lock = threading.Lock()

        # ROS 控制变量
        self._ros_thread = None
        self._ros_running = False

    def start_ros(self):
        """启动 rclpy 并在单独线程中 spin ROS 节点。"""
        def ros_thread_fn():
            rclpy.init()
            self._ros_node = TRJointSubscriber(self)
            self._ros_running = True
            try:
                rclpy.spin(self._ros_node)
            except Exception as e:
                print(f"ROS spin exception: {e}")
            finally:
                try:
                    self._ros_node.destroy_node()
                except Exception:
                    pass
                rclpy.shutdown()
                self._ros_running = False

        self._ros_thread = threading.Thread(target=ros_thread_fn, daemon=True)
        self._ros_thread.start()
        # 等待节点启动
        timeout = 2.0
        t0 = time.time()
        while not getattr(self, '_ros_running', False) and (time.time() - t0) < timeout:
            time.sleep(0.01)
        print("ROS2 subscriber thread started")

    def stop_ros(self):
        """请求 ROS 停止"""
        if self._ros_running:
            try:
                rclpy.shutdown()
            except Exception:
                pass
            self._ros_running = False
            if self._ros_thread:
                self._ros_thread.join(timeout=1.0)

    def emergency_callback(self, station_nr):
        print(f"紧急事件来自从站 {station_nr}")
        self._master.state_check(pysoem.INIT_STATE)
        print("系统已重置至INIT状态")

    def elmo_setup(self, slave_pos):
        slave = self._master.slaves[slave_pos]
        self.slave_pos = slave_pos
        slave.dc_sync(act=True, sync0_cycle_time=1_000_000)  # 1ms
        print("01")

    def enable_motor(self, slave_pos):
        slave = self._master.slaves[slave_pos]
        control_sequence = [
            (0x06, "Shutdown", 0b00100001, 0b01101111),
            (0x07, "Switch on", 0b00100011, 0b01101111),
            (0x0F, "Enable operation", 0b00100111, 0b01101111)
        ]

        # 设置PP模式 (0x01)
        slave.sdo_write(MODE_OF_OPERATION, 0, struct.pack('<b', 0x01))
        actual_mode = struct.unpack('<b', slave.sdo_read(0x6061, 0))[0]
        print(f"运动模式为：{actual_mode}")

        status_bytes = slave.sdo_read(STATUSWORD, 0)
        status_word = struct.unpack('<H', status_bytes)[0]
        mask_code = 0b00001000
        masked_status = status_word & mask_code
        if masked_status == 0b00001000:
            slave.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x80))
        print(f"状态字: {status_word:08b}")

        slave.sdo_write(PROFILE_VELOCITY, 0, struct.pack('<I', profile_velocity))
        if slave_pos < 2:
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<I', x_profile_acceleration))
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<I', x_profile_deceleration))
        else:
            slave.sdo_write(PROFILE_ACCELERATION, 0, struct.pack('<I', profile_acceleration))
            slave.sdo_write(PROFILE_DECELERATION, 0, struct.pack('<I', profile_deceleration))

        for cw, desc, expected_status, mask_code in control_sequence:
            print(f"{desc} (0x{cw:02X})")
            slave.sdo_write(CONTROLWORD, 0, struct.pack('<H', cw))
            time.sleep(0.1)
            status_bytes = slave.sdo_read(STATUSWORD, 0)
            status_word = struct.unpack('<H', status_bytes)[0]
            masked_status = status_word & mask_code
            print(f"状态字: {status_word:08b}")
            print(f"期望状态{expected_status:08b}")
            if masked_status != expected_status:
                expected_binary = f"{expected_status:08b}"
                if cw == 0x80:
                    continue
                raise RuntimeError(
                    f"状态机异常！阶段 [{desc}]\n"
                    f"当前状态: {masked_status:08b}\n"
                    f"预期模式: {expected_binary} (x01x {expected_binary[-4:]})"
                )
            else:
                print(f"状态校验通过: {desc} ")
            time.sleep(0.01)

        print("10 电机已完全使能")

    def run_trajectory(self):

        """
        执行轨迹。轨迹点现在来自 self.trajectory_points （由 ROS 回调更新）。
        始终只执行队列中的最新点（队列长度被保证为 0 或 1）。
        """
        slave_x1 = self._master.slaves[0]
        slave_x2 = self._master.slaves[1]
        slave_y = self._master.slaves[2]
        slave_z = self._master.slaves[3]
        all_slaves = [slave_x1, slave_x2, slave_y, slave_z]

        current_target_point = None
        last_point_time = 0
        cycle_counter = 0
        follow_err = 0

        try:
            print("\nStarting trajectory execution...")
            while True:
                cycle_start_time = time.time()

                # 每次循环尝试读取最新目标（线程安全）
                with self.trajectory_lock:
                    if self.trajectory_points:
                        # 取但不弹出，保持队列语义（外部可继续替换）
                        newest = self.trajectory_points[-1]
                    else:
                        newest = None

                if newest is not None:
                    # 如果和当前目标不同则替换并下发
                    if (current_target_point is None) or (
                        newest['x'] != current_target_point['x'] or
                        newest['y'] != current_target_point['y'] or
                        newest['z'] != current_target_point['z']
                    ):
                        current_target_point = newest.copy()
                        last_point_time = time.time()
                        print(f"\n[{last_point_time:.2f}s] New Target Point -> X: {current_target_point['x']}, Y: {current_target_point['y']}, Z: {current_target_point['z']}")

                        # 通过 SDO 写入目标位置
                        slave_x1.sdo_write(TARGET_POSITION, 0, struct.pack('<i', current_target_point['x']))
                        slave_x2.sdo_write(TARGET_POSITION, 0, struct.pack('<i', current_target_point['x']))
                        slave_y.sdo_write(TARGET_POSITION, 0, struct.pack('<i', current_target_point['y']))
                        slave_z.sdo_write(TARGET_POSITION, 0, struct.pack('<i', current_target_point['z']))

                        # 发送控制字 0x3F 以立即切换到新目标
                        for s in all_slaves:
                            s.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x3F))


                control_words = {s: 0x0F for s in all_slaves}
                for slave in all_slaves:
                    try:
                        slave.sdo_write(CONTROLWORD, 0, struct.pack('<H', control_words[slave]))
                    except Exception:
                        pass


                # 这里我们做一次 send/receive 来读取 input 状态
                if current_target_point is None:
                    # 如果还没有收到新点，就用默认零点
                    if newest is None:
                        newest = {'x': 0, 'y': 0, 'z': 0}  # 初始默认值
                    current_target_point = newest.copy()

                # ===== 下发 PDO 输出 =====
                try:
                    input_x2_as_int16 = struct.pack('<HbiH', 0x0F, 0x01, current_target_point['x'], 0x00)
                    input_x1_as_int16 = struct.pack('<HbiH', 0x0F, 0x01, current_target_point['x'], 0x00)
                    self._master.slaves[0].output = input_x1_as_int16
                    self._master.slaves[1].output = input_x2_as_int16
                    input_y_as_int16 = struct.pack('<HbiH', 0x0F, 0x01, current_target_point['y'], 0x00)
                    input_z_as_int16 = struct.pack('<HbiH', 0x0F, 0x01, current_target_point['z'], 0x00)
                    self._master.slaves[2].output = input_y_as_int16
                    self._master.slaves[3].output = input_z_as_int16
                except Exception as e:
                    print("Packing outputs exception:", e)

                try:
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
                except Exception as e:
                    print("Processdata send/receive exception:", e)

                # 固定周期控制
                elapsed = time.time() - cycle_start_time
                sleep_time = 0.02 - elapsed  # 20ms
                if sleep_time > 0:
                    time.sleep(sleep_time)

                # 继续原有 processdata 写法（pack 控制字和目标位置）
                try:
                    input_x2_as_int16 = struct.pack('<HbiH', control_words[slave_x1], 0x01, current_target_point['x'], 0x00)
                    input_x1_as_int16 = struct.pack('<HbiH', control_words[slave_x2], 0x01, current_target_point['x'], 0x00)
                    self._master.slaves[0].output = input_x1_as_int16
                    self._master.slaves[1].output = input_x2_as_int16
                    input_z_as_int16 = struct.pack('<HbiH', control_words[slave_z], 0x01, current_target_point['z'], 0x00)
                    input_y_as_int16 = struct.pack('<HbiH', control_words[slave_y], 0x01, current_target_point['y'], 0x00)
                    self._master.slaves[2].output = input_y_as_int16
                    self._master.slaves[3].output = input_z_as_int16
                except Exception as e:
                    print("Packing outputs exception:", e)

                try:
                    self._master.send_processdata()
                    self._master.receive_processdata(2000)
                except Exception as e:
                    print("Processdata send/receive exception:", e)

                # 读取并显示状态（通过PDO读取，若失败则通过sdo读取）
                try:
                    output_x1_as_bytes = self._master.slaves[0].input
                    output_x1_as_int16 = struct.unpack('<H H B i H i i I ', output_x1_as_bytes)
                    x1_actual_pos = output_x1_as_int16[3]
                    x1_status_word = output_x1_as_int16[1]
                except Exception:
                    x1_actual_pos = struct.unpack('<i', self._master.slaves[0].sdo_read(ACTUAL_POSITION, 0))[0]
                    x1_status_word = struct.unpack('<H', self._master.slaves[0].sdo_read(STATUSWORD, 0))[0]

                try:
                    output_x2_as_bytes = self._master.slaves[1].input
                    output_x2_as_int16 = struct.unpack('<H H B i H i i I ', output_x2_as_bytes)
                    x2_actual_pos = output_x2_as_int16[3]
                    x2_status_word = output_x2_as_int16[1]
                except Exception:
                    x2_actual_pos = struct.unpack('<i', self._master.slaves[1].sdo_read(ACTUAL_POSITION, 0))[0]
                    x2_status_word = struct.unpack('<H', self._master.slaves[1].sdo_read(STATUSWORD, 0))[0]

                try:
                    output_y_as_bytes = self._master.slaves[2].input
                    output_y_as_int16 = struct.unpack('<H H B i H i i I I', output_y_as_bytes)
                    y_actual_pos = output_y_as_int16[3]
                    y_status_word = output_y_as_int16[1]
                except Exception:
                    y_actual_pos = struct.unpack('<i', self._master.slaves[2].sdo_read(ACTUAL_POSITION, 0))[0]
                    y_status_word = struct.unpack('<H', self._master.slaves[2].sdo_read(STATUSWORD, 0))[0]

                try:
                    output_z_as_bytes = self._master.slaves[3].input
                    output_z_as_int16 = struct.unpack('<H H B i H i i I I', output_z_as_bytes)
                    z_actual_pos = output_z_as_int16[3]
                    z_status_word = output_z_as_int16[1]
                except Exception:
                    z_actual_pos = struct.unpack('<i', self._master.slaves[3].sdo_read(ACTUAL_POSITION, 0))[0]
                    z_status_word = struct.unpack('<H', self._master.slaves[3].sdo_read(STATUSWORD, 0))[0]

                if cycle_counter % 10 == 0:
                    pos = {}
                    status = {}
                    for i, slave in enumerate(all_slaves):
                        name = self._expected_slave_mapping[i].slave_name
                        pos[name] = struct.unpack('<i', slave.sdo_read(ACTUAL_POSITION, 0))[0]
                        status[name] = struct.unpack('<H', slave.sdo_read(STATUSWORD, 0))[0]
                        print(f"{name} | Pos: {pos[name]:>10d} | Status: {status[name]:016b}")

                # 检查到达条件
                if current_target_point is not None:
                    if (abs(x1_actual_pos - current_target_point['x']) < 1000 and
                        abs(x2_actual_pos - current_target_point['x']) < 1000 and
                        abs(y_actual_pos - current_target_point['y']) < 1000 and
                        abs(z_actual_pos - current_target_point['z']) < 500):
                        print("Target reached .")


                # 简化的跟随误差处理（保留原逻辑的大意）
                if abs(x1_actual_pos - x2_actual_pos) > 5000000 and follow_err == 0:
                    follow_err = 1
                    if current_target_point and current_target_point['x'] - x1_actual_pos < 0:
                        offset_pos = x1_actual_pos - 100000000
                    else:
                        offset_pos = x1_actual_pos + 100000000

                    offset_pos = max(0, min(offset_pos, X_CNT_2_DIG))

                    slave_x1.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x0F))
                    slave_x2.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x0F))
                    slave_x2.sdo_write(TARGET_POSITION, 0, struct.pack('<i', offset_pos))
                    slave_x1.sdo_write(TARGET_POSITION, 0, struct.pack('<i', offset_pos))
                    slave_x1.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x3F))
                    slave_x2.sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x3F))
                    print("x轴出问题啦！")

                cycle_counter += 1
                elapsed = time.time() - cycle_start_time
                sleep_time = 0.02 - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            print("\nStopped by user.")
        finally:
            print("Disabling all motors...")
            for i in range(len(self._master.slaves)):
                try:
                    self._master.slaves[i].sdo_write(CONTROLWORD, 0, struct.pack('<H', 0x06))
                except Exception as e:
                    print(f"Could not disable motor {i}: {e}")

    def run(self):
        # 启动 ROS2 subscriber
        try:
            self.start_ros()
        except Exception as e:
            print("Failed to start ROS2 thread:", e)
        #开启主站
        self._master.open(self._ifname)
        if self._master.config_init() > 0:
            print("{} slaves found and configured".format(len(self._master.slaves)))

            for i, slave in enumerate(self._master.slaves):
                assert (slave.man == self.BECKHOFF_VENDOR_ID)
                assert (slave.id == self._expected_slave_mapping[i].product_code)
                slave.config_func = self._expected_slave_mapping[i].config_func
                slave.add_emergency_callback(self.emergency_callback)

            self._master.config_map()
            self._master.state = pysoem.SAFEOP_STATE
            self._master.write_state()
            time.sleep(1)
            if self._master.state_check(pysoem.SAFEOP_STATE, 1000000) != pysoem.SAFEOP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if slave.state != pysoem.SAFEOP_STATE:
                        print('{} did not reach SAFEOP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                        print(slave.state)
                raise Exception('not all slaves reached SAFEOP state')
            print("all slaves reached SAFEOP state")

            self._master.state = pysoem.OP_STATE
            self._master.write_state()
            time.sleep(2)
            self._master.state_check(pysoem.OP_STATE, 1000000)
            if self._master.state != pysoem.OP_STATE:
                self._master.read_state()
                for slave in self._master.slaves:
                    if not slave.state == pysoem.OP_STATE:
                        print('{} did not reach OP state'.format(slave.name))
                        print('al status code {} ({})'.format(hex(slave.al_status),
                                                              pysoem.al_status_code_to_string(slave.al_status)))
                        print(slave.state)
            print("all slaves reached OP state")
            try:
                self.enable_motor(0)
                self.enable_motor(1)
                self.enable_motor(2)
                self.enable_motor(3)

                # 初始 trajectory_points 可保留若干默认点或为空
                # 这里我们不填充初始点，等待 ROS 发布新的指令
                self.run_trajectory()

            except KeyboardInterrupt:
                print('stopped')
            finally:
                self._master.state = pysoem.INIT_STATE
                self._master.write_state()
                self._master.close()
                # 停止 ROS
                try:
                    self.stop_ros()
                except Exception:
                    pass
        else:
            print('slaves not found')

if __name__ == '__main__':
    print('minimal_example_with_ros2')

    if len(sys.argv) > 1:
        MinimalExample(sys.argv[1]).run()
    else:
        print('usage: minimal_example ifname')
        sys.exit(1)
