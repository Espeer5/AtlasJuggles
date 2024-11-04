"""Utility functions and objects used to help Atlas keep track of juggling 
trajectories and information"""

import numpy as np
import random
from math import acos
import copy

from code.TransformHelpers import Rote
from code.constants import *

# A map of frames used to hit the ball to the position they hit it, and their 
# surface normals in their local frame
FRAME_HIT_MAP = {
    "head" : (np.array([[0.1], [-0.02], [.835]]), np.array([[0.0], [0.0], [1.0]])),
    "r_foot" : (np.array([[0.4], [-0.3], [-0.4]]), np.array([[0.0], [0.0], [1.0]])),
    "r_hand" : (np.array([[0.4], [-0.7], [.38]]), np.array([[1.0], [0.0], [0.0]])),
    "l_foot" : (np.array([[0.4], [0.3], [-0.4]]), np.array([[0.0], [0.0], [1.0]])),
    "l_hand" : (np.array([[0.54], [0.62], [.35]]), np.array([[0.0], [0.0], [1.0]])),
    "r_lleg" : (np.array([[0.4], [-0.13], [-0.4]]), np.array([[0.0], [0.0], [1.0]])),
    "l_lleg" : (np.array([[0.4], [0.13], [-0.1]]), np.array([[0.0], [0.0], [1.0]])),
}

class HitPoint():
    """"A class used to store information about a point where the ball is 
    hit by Atlas."""

    def __init__(self, time, position, hit_frame, normal_v):
        """Initializes a HitPoint object with the given time, position, and 
        velocity."""
        self.hit_frame = hit_frame
        self.normal_v = normal_v
        self.time = time
        self.position = position
        self.velocity_f = None
        self.velocity_b = None
        self.rot_axis = None
        self.alpha = None


class HitPlan():
    """A set of HitPoints that Atlas is intending to traverse"""
    
    def __init__(self, start, link_1, link_2, dt1, dt2):
        """Initializes a HitPlan object"""
        self.prev_goal = HitPoint(0.0, start, None, None)
        self.prev_goal.velocity_b = np.array([[0.0], [0.0], [0.0]])
        self.curr_goal = HitPoint(dt1, FRAME_HIT_MAP[link_1][0], link_1,
                                  FRAME_HIT_MAP[link_1][1])
        self.next_goal = HitPoint(dt1+dt2, FRAME_HIT_MAP[link_2][0], link_2,
                                  FRAME_HIT_MAP[link_2][1])

    def calculate_desired_exit_velocity(self):
        desired_dt = self.next_goal.time - self.curr_goal.time

        x0, y0, z0 = self.curr_goal.position
        x1, y1, z1 = self.next_goal.position

        delta_x = x1 - x0 
        delta_y = y1 - y0
        delta_z = z1 - z0

        vx = delta_x / desired_dt
        vy = delta_y / desired_dt
        vz = (delta_z - (1/2 * GRAV * desired_dt ** 2)) / desired_dt

        return np.array([[vx], [vy], [vz]]).reshape(3, 1)

    def get_v_curr(self, v_i, dt): 
        v_curr = v_i
        v_curr[2] = v_i[2] + GRAV * dt
        return v_curr
    
    def get_kicking_velocity(self):
        # desired_exit_velocity = (kicking_velocity + ball_velocity) * elasticity
        d_exit_velocity = self.calculate_desired_exit_velocity()

        norm = d_exit_velocity / np.linalg.norm(d_exit_velocity)
        ball_velo = self.get_v_curr(self.prev_goal.velocity_b,
                             self.curr_goal.time - self.prev_goal.time)

        ball_velo_on_axis = norm * (ball_velo * norm)

        # kicking_velocity = (desired_exit_velocity / elasticity) - ball_velocity
        kicking_velocity = (d_exit_velocity / BALL_ELAS) + ball_velo_on_axis
        return kicking_velocity.reshape(3, 1)
    
    def plan_curr(self):
        self.curr_goal.velocity_b = self.calculate_desired_exit_velocity()
        self.curr_goal.velocity_f = self.get_kicking_velocity()
        v_bounce = self.curr_goal.velocity_b.reshape(3,)
        v_unit = v_bounce / np.linalg.norm(v_bounce)
        z_i = self.curr_goal.normal_v.reshape(3,)
        e_axis = np.cross(z_i, v_unit)
        alpha_e = acos(np.dot(z_i, v_unit)/np.linalg.norm(z_i)*np.linalg.norm(v_unit))
        self.curr_goal.rot_axis = e_axis
        self.curr_goal.alpha = alpha_e

    def add_next(self, next_goal):
        """Adds the next goal to the HitPlan. Replaces the last prev goal with 
        the current goal"""
        self.prev_goal = copy.deepcopy(self.curr_goal)
        self.curr_goal = copy.deepcopy(self.next_goal)
        self.next_goal = next_goal
        self.plan_curr()

    def add_next_link(self, dt, hit_link=None):
        """Adds a random next goal to the HitPlan if no link specified, or 
        the specified link if one is given"""
        if not hit_link:
            while hit_link == self.next_goal.hit_frame or hit_link == None:
                hit_link = random.choice(list(FRAME_HIT_MAP.keys()))
        next_goal = HitPoint(self.next_goal.time + dt,
                                FRAME_HIT_MAP[hit_link][0],
                                hit_link,
                                FRAME_HIT_MAP[hit_link][1])
        self.add_next(next_goal)
