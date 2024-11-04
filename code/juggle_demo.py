"""The final demo for ME/CS 133a in which Atlas juggles a ball."""

import rclpy
from code.ROS_utils import *
from code.juggle_utils import *
from code.constants import *
import numpy as np
from itertools import chain
import random as rand

from rclpy.node                 import Node
from rclpy.time                 import Duration
from hw5code.KinematicChain     import KinematicChain
from code.TrajectoryUtils       import *
from code.TransformHelpers      import *

# Trajectory class
class Trajectory():
    # Initialization
    def __init__(self, node):
        # Get a kinematic chain for each link used to strike the ball
        self.chains = {frame : KinematicChain(node, STRIKE_FRAMES[frame][0], frame, 
                                         STRIKE_FRAMES[frame][1]) for frame in 
                                         STRIKE_FRAMES.keys()}
        
        # Fixed shift from world to pelvis
        self.p_pelvis = pxyz(0.0, 0.0, 0.81863547)

        self.time_to_hit = .5 # time between ball hits
        
        # Ball radius, position, velocity, and acceleration
        self.ball_radius = BALL_RAD
        start_height = 0.5 * -GRAV * self.time_to_hit ** 2 + .837
        self.p0ball = np.array([[0.1], [-0.02], [start_height]]) + self.p_pelvis
        self.v0ball = np.array([0.0, 0.0, 0.0]).reshape((3,1))

        # Save the positions and velocities for each point at each time step
        self.pball = self.p0ball
        self.vball = self.v0ball
        self.aball = np.array([0.0, 0.0, GRAV]).reshape((3,1))
        self.q_s = {joint : 0 for joint in self.jointnames()}

        self.q_s["r_leg_kny"] = 0.8
        self.q_s["l_leg_kny"] = 0.8
        self.q_s["l_leg_hpy"] = -0.4
        self.q_s["r_leg_hpy"] = -0.4
        self.q_s["l_arm_elx"] = 0.1
        self.q_s["r_arm_elx"] = -0.1
        self.q_s["l_arm_shx"] = -1.4
        self.q_s["r_arm_shx"] = 1.4

        # Get the rest positions for each of the chain tips
        self.rest_pos = {frame : self.chains[frame].fkin(np.array([self.q_s[joint] for joint in self.jointnames(frame)]).reshape(len(self.jointnames(frame))))[0] for frame in
                           STRIKE_FRAMES.keys()}


        self.qdot = np.zeros(len(self.jointnames()))

        # Trajectory variables
        self.lamb = 20
        self.hit_timer = 0
        f_s = list(STRIKE_FRAMES.keys())
        self.plan = HitPlan(self.p0ball, "head", rand.choice(f_s), self.time_to_hit, 1.2)

        # Plan the first hit
        self.plan.plan_curr()

    def jointnames(self, chain_frame=None):
        if not chain_frame:
            return list(chain.from_iterable([STRIKE_FRAMES[chain][1] for chain in 
                                             STRIKE_FRAMES.keys()]))
        else:
            return STRIKE_FRAMES[chain_frame][1]
        
    def evaluate(self, t, dt):
        # Determine the goal position and velocity for each chain tip
        self.hit_timer += dt
        goal_ps = {frame : self.rest_pos[frame] for frame in 
                   STRIKE_FRAMES.keys()}
        goal_ps[self.plan.curr_goal.hit_frame] = self.plan.curr_goal.position
        goal_Rs = {frame : (np.zeros((3, 1)), np.zeros((3, 1))) for frame in
                   STRIKE_FRAMES.keys()}
        goal_Rs[self.plan.curr_goal.hit_frame] = (self.plan.curr_goal.rot_axis,
                                                    self.plan.curr_goal.alpha)
        goal_vs = {frame : np.zeros((3, 1)) for frame in STRIKE_FRAMES.keys()}
        goal_vs[self.plan.curr_goal.hit_frame] = self.plan.curr_goal.velocity_b
        start_p = {frame : self.rest_pos[frame] for frame in STRIKE_FRAMES.keys()}
        if self.plan.prev_goal.hit_frame:
            start_p[self.plan.prev_goal.hit_frame] = self.plan.prev_goal.position
        qdot = np.concatenate([get_qd(self.chains[frame], start_p[frame],
                                      goal_ps[frame], np.zeros((3, 1)), 
                                      goal_vs[frame], goal_Rs[frame][0],
                                      goal_Rs[frame][1], [self.q_s[joint] for
                                                joint in self.jointnames(frame)],
                                      self.time_to_hit, self.hit_timer % self.time_to_hit) 
                                      for frame in STRIKE_FRAMES.keys()])
        qdot = qdot.flatten().tolist()
        temp_joint_names = self.jointnames()
        def indices(lst, item):
            return [i for i, x in enumerate(lst) if x == item]
        target = self.plan.curr_goal.hit_frame

        start_indices = {}
        total_joints = 0
        for chain in self.chains.keys():
            start_indices[chain] = total_joints
            total_joints += len(self.jointnames(chain))
        assert(total_joints == len(self.jointnames()))
        target_indices = list(range(start_indices[target] + 1, 1 + start_indices[target] + len(self.jointnames(target))))

        dups = dict((x, indices(self.jointnames(), x)) for x in set(self.jointnames()) if self.jointnames().count(x) > 1)
        toDel = []
        for dup in dups.keys():
            toKeep = [dups[dup][i] for i in range(len(dups[dup])) if dups[dup][i] in target_indices]
            if len(toKeep) == 0:
                toKeep = [dups[dup][0]]
            toDel += [dup for dup in dups[dup] if dup not in toKeep]
        qdot = [qdot[i] for i in range(len(qdot)) if i not in toDel]
        temp_joint_names = [temp_joint_names[i] for i in range(len(temp_joint_names)) if i not in toDel]

        qdot = np.array(qdot).reshape(len(qdot), 1)
        q = np.array([self.q_s[joint] for joint in temp_joint_names]).reshape(len(temp_joint_names), 1) + qdot * dt
        self.q_s = {joint : q[i][0] for i, joint in enumerate(temp_joint_names)}
        return q, qdot, temp_joint_names
    
    def evaluate_ball(self, t, dt):
        # Find the position and orientation of the joint targeting the ball
        target = self.plan.curr_goal.hit_frame
        phead, _, _, _ = self.chains[target].fkin(np.array([self.q_s[joint] for joint in self.jointnames(target)]).reshape(len(self.jointnames(target)), 1))
        Rhead = Rote(self.plan.curr_goal.rot_axis, self.plan.curr_goal.alpha)

        # must be in this order
        self.vball += dt * self.aball
        self.pball += dt * self.vball

        # evaluate the ball trajectory at the given time
        if np.linalg.norm(self.pball - phead - self.p_pelvis) < BALL_RAD:
            self.time_to_hit = 1.05
            self.pball[2,0] += self.ball_radius
            # find the normal vector
            norm = Rhead @ self.plan.curr_goal.normal_v
            norm = norm / np.linalg.norm(norm)

            self.hit_timer = 0

            self.vball = self.plan.curr_goal.velocity_b
            if np.linalg.norm(self.pball - self.plan.curr_goal.position + self.p_pelvis) > 5e-2:
                self.pball = self.plan.curr_goal.position + self.p_pelvis
            self.plan.add_next_link(self.time_to_hit)

        return self.pball, self.vball


# Juggle Node Class
class JuggleNode(Node):
    # Initialization
    def __init__(self, name, rate):
        # Initialize ROS node
        super().__init__(name)

        # Create a trajectory object for path planning
        self.traj = Trajectory(self)

        # Create a publisher for the robot state
        self.robot_pub = robo_publisher(self, JOINT_NAMES)

        # Create a publisher for the ball
        self.ball_pub = ball_publisher(self, BALL_RAD)

        # Set up timing and create the timer for the main loop
        self.dt = 1.0 / float(rate)
        self.t = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)
        self.create_timer(self.dt, self.update)
        self.get_logger().info(f"Running with dt of {self.dt} seconds ({rate}Hz)")

    # Shutdown
    def shutdown(self):
        # Destroy the node including cleaning up timer
        self.destroy_node()

    # Return the current time in ROS format
    def now(self):
        return self.get_clock().now()
    
    # Update with new joint state and ball position each timestep
    def update(self):
        # Update the time
        self.t += self.dt

        # Get the current joint state and pelvis shift
        q_r, qdot_r, j_n = self.traj.evaluate(self.t, self.dt)

        # Get the current ball state
        pball, _ = self.traj.evaluate_ball(self.t, self.dt)

        # Assemble the full joint state from the subset evaluated by the traj
        q = [0.0 for _ in range(len(JOINT_NAMES))]
        qdot = [0.0 for _ in range(len(JOINT_NAMES))]

        for joint_name, curr_q, curr_qdot in zip(j_n, q_r, qdot_r):
            index = JOINT_NAMES.index(joint_name)
            q[index] = curr_q[0]
            qdot[index] = curr_qdot[0]

        T_pelvis = T_from_Rp(Reye(), self.traj.p_pelvis)

        # Publish the robot state
        self.robot_pub.publish(T_pelvis, q, qdot)

        # Publish the ball state
        self.ball_pub.publish(pball)


# Main executable
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    # generator = GeneratorNode('generator', 100, Trajectory)
    node = JuggleNode('balls', 100)

    # Spin, until done
    # generator.spin()
    rclpy.spin(node)

    # Shutdown the node and ROS.
    # generator.shutdown() 
    node.shutdown()

    rclpy.shutdown()


if __name__ == "__main__":
    main()

