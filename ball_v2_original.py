from omni.isaac.kit import SimulationApp  # 初始化Simulation App, 4.0.0 中SimulationApp 需要用 omni.isaac.kit 导入
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicSphere, GroundPlane
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.viewports import set_camera_view
import numpy as np
import carb
from pxr import UsdPhysics, Sdf, Usd

# 创建一个World对象
world = World()

# 添加地面
# ground = GroundPlane()
ground = GroundPlane(prim_path="/World/Ground")
world.scene.add(ground)

# 定义网球的参数
radius = 0.033  # 网球半径，单位：米（标准网球直径为6.7cm）
mass = 0.057  # 网球质量，单位：千克
initial_speed = 30.0  # 初速度，单位：米/秒
launch_angle_deg = 45.0  # 发射角度，单位：度

# 将角度转换为弧度
launch_angle_rad = np.deg2rad(launch_angle_deg)

# 计算初速度的分量
initial_velocity = np.array([
    initial_speed * np.cos(launch_angle_rad),
    0.0,  # 假设在x-z平面内发射，y方向速度为0
    initial_speed * np.sin(launch_angle_rad)
])

# 添加网球（动态球体）
tennis_ball = DynamicSphere(
    prim_path="/World/tennis_ball",
    name="tennis_ball",
    position=np.array([0.0, radius + 0.01, 0.0]),  # 略高于地面，避免初始碰撞
    radius=radius,
    color=np.array([1.0, 0.2, 0.2]),  # 红色
    mass=mass
)
world.scene.add(tennis_ball)

# 设置物理属性
material_path = f"{tennis_ball.prim_path}/physicsMaterial"
stage = world.scene.stage
material_prim = stage.DefinePrim(material_path, "Material")
material_api = UsdPhysics.MaterialAPI.Apply(material_prim)
material_api.CreateStaticFrictionAttr().Set(0.5)
material_api.CreateDynamicFrictionAttr().Set(0.5)
material_api.CreateRestitutionAttr().Set(0.6)  # 弹性系数

# 将材质应用于网球
tennis_ball_prim = stage.GetPrimAtPath(tennis_ball.prim_path)
tennis_ball_prim.GetRelationship("physics:material:binding").SetTargets([Sdf.Path(material_path)])

# 启动世界
world.reset()

# 设置初始速度
tennis_ball.set_linear_velocity(initial_velocity)

# 设置相机视角（可选）
# set_camera_view(position=np.array([-1.0, 1.0, 1.0]),
#                target=np.array([0.0, 0.0, 0.0]))

# 运行仿真
simulation_steps = 240  # 运行240步，相当于4秒（假设60步/秒）
while True:
    world.step(render=True)

# 关闭SimulationApp
simulation_app.close()
