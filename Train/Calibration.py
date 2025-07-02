import math
import numpy as np
from isaacsim.util.debug_draw import _debug_draw
from carb import Float3, ColorRgba
# 正确的输入格式
def point_draw(position, color=[1.0, 0.0, 0.0, 1.0], size=10):
    # 转换成 ColorRgba 对象（不再包外层 list）
    draw = _debug_draw.acquire_debug_draw_interface()
    pos_list = [Float3(*position)]
    color_list = [ColorRgba(*color)]
    size_list = [size]

    draw.clear_points()
    draw.draw_points(pos_list, color_list, size_list)

def line_draw(start, end, color=[1.0, 0.0, 0.0, 1.0], size=2.0):
    draw = _debug_draw.acquire_debug_draw_interface()
    draw.clear_lines()
    draw.draw_lines(
        [Float3(*start)],
        [Float3(*end)],
        [ColorRgba(*color)],
        [size]
    )

def calculate_racket_center(world_move_x, world_move_y, world_move_z, theta):
    # 初始化变量
    x = world_move_x
    y = world_move_y
    z = world_move_z
    theta = theta

    # 计算相对坐标
    xr = 2.32511 - x
    yr = 2.72915 - y
    zr = 0.33975 + z

    # 计算半径
    R = 2.44 - 1.72915

    # 计算目标点坐标（使用角度输入）
    # xq = xr + R * math.cos(math.radians(theta)
    xq = xr + R
    # yq = yr + R * math.sin(math.radians(theta))
    yq = yr
    zq = zr + 0.0925
    hitting_center = [xq, yq, zq, theta]
    point_draw(hitting_center[:3], [1.0, 1.0, 0.8, 1.0], 10)
    return hitting_center

def calculate_coordinates(world_move_x, world_move_y, world_move_z, theta,ball_x, ball_y, ball_z):

# 初始化变量
    x = world_move_x
    y = world_move_y
    z = world_move_z
    alpha = theta

    # 计算相对坐标
    xr = 2.32511 - x
    yr = 2.72915 - y
    zr = 0.33975 + z

    # 计算半径
    R = 2.44 - 1.72915


    # 计算目标点坐标（使用角度输入）
    xq = xr + R * math.cos(math.radians(alpha))
    yq = yr + R * math.sin(math.radians(alpha))
    zq = zr + 0.0925
    racket_center = [xq, yq, zq]
    point_draw(racket_center, [1.0, 0.0, 0.8, 1.0], 10)
    
    # print(f"{xq:.5f} {yq:.5f} {zq:.5f}")

    # 基准点坐标
    xb = ball_x
    yb = ball_y
    zb = ball_z
    ball_center = [xb, yb, zb]
    line_draw(racket_center, ball_center, [0.0, 1.0, 0.0, 1.0], 2.0)

    linear_distance = math.sqrt((xb - xq) ** 2 + (yb - yq) ** 2 + (zb - zq) ** 2)
    
    return linear_distance