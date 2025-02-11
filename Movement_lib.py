from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
import math
import numpy as np

def reset(Robot,initial_position=np.array([0,0,0,0])):
    Robot.get_articulation_controller().apply_action(
        ArticulationAction(joint_positions=initial_position, joint_velocities=[1.5,1.5,0.6,20])
    )

def hit_tennisball_forehand(Robot,current_position):
    Robot.get_articulation_controller().apply_action(
            ArticulationAction(joint_positions=[current_position[0], current_position[1], current_position[2], 0/180*2*math.pi],joint_velocities=[1.5,1.5,0.6,20])#'X_Pris', 'Z_Pris_H', 'Z_Pris_V', 'Racket_Pev', bias x+0.7272, z+0.17412, joint_velocities didnt work, should set controller mode to mixed
            )