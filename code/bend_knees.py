'''pirouetteandwave.py

   This is a demo for moving/placing an ungrounded robot and moving joints.

   In particular, imagine a humanoid robot.  This moves/rotates the
   pelvis frame relative to the world.  And waves an arm.

   Node:        /pirouette
   Publish:     /joint_states               sensor_msgs/JointState
   Broadcast:   'pelvis' w.r.t. 'world'     geometry_msgs/TransformStamped

'''

import rclpy
import numpy as np

from math import pi, sin, cos, acos, atan2, sqrt, fmod, exp

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

from code.TransformHelpers     import *


#
#   Atlas Joint Names
#
jointnames = ['l_leg_hpx', 'l_leg_hpy', 'l_leg_hpz',
              'l_leg_kny',
              'l_leg_akx', 'l_leg_aky',

              'r_leg_hpx', 'r_leg_hpy', 'r_leg_hpz',
              'r_leg_kny',
              'r_leg_akx', 'r_leg_aky',

              'back_bkx', 'back_bky', 'back_bkz',
              'neck_ry',

              'l_arm_elx', 'l_arm_ely',
              'l_arm_shx', 'l_arm_shz',
              'l_arm_wrx', 'l_arm_wry', 'l_arm_wry2',

              'r_arm_elx', 'r_arm_ely',
              'r_arm_shx', 'r_arm_shz',
              'r_arm_wrx', 'r_arm_wry', 'r_arm_wry2']


#
#   Trajectory Class
#
class Trajectory():
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, 'world', 'tip', self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 45.5675, 0, -93.1349, 0, 0, 46.5675]).reshape((7,1)))
        self.p0 = np.array([0.0, 0.7, 0.6]).reshape((3,1))
        self.R0 = Reye()

        self.pleft  = np.array([0.3, 0.5, 0.15]).reshape((-1,1))
        self.pright = np.array([-0.3, 0.5, 0.15]).reshape((-1,1))

        # Initialize the current/starting joint position.
        self.q  = self.q0
        self.lamda = 20

        # Problem Set 7: Setup the condition number publisher
        self.pub = node.create_publisher(Float64, '/condition', 10)


    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ['theta1', 'theta2', 'theta3', 'theta4', 'theta5', 'theta6', 'theta7']

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        Rd = Reye()
        wd = np.array([[0.0], [0.0], [0.0]]).reshape(3, 1)

        pd = np.array([[0.0], [0.95 - 0.25 * np.cos(t)], [0.60 + 0.25 * np.sin(t)]])
        vd = np.array([[0.0], [0.25 * np.sin(t)], [0.25 * np.cos(t)]])
        
        # FIXME: REUSE THE PREVIOUS INVERSE KINEMATICS.
        qlast = self.q

        # Compute the old forward kinematics.
        (p, R, Jv, Jw) = self.chain.fkin(qlast)

        J = np.vstack((Jv, Jw))
        
        xdot = np.vstack((vd, wd))
        error = np.vstack((ep(pd, p), eR(Rd, R)))
        error *= self.lamda

        qdot = np.linalg.pinv(J) @ (xdot + error)

        ### 7.4b ###
        JT =  np.transpose(J)
        gamma = 1.0
        J_winv = JT @ np.linalg.pinv((J @ JT + gamma**2 * np.eye(6)))
        qdot = J_winv @ (xdot + error)
        ###

        ### 7.4c ###
        qdot_secondary = np.array([[0.0], [0.0], [0.0], [np.float(-np.pi/2 - qlast[3])], [0.0], [0.0], [0.0]]).reshape(7, 1)
        secondary = (np.eye(7) - np.linalg.pinv(J) @ J) @ qdot_secondary
        qdot += 1000 * secondary
        ###

        q = qlast + dt * qdot
        self.q = q

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS.
    rclpy.init(args=args)

    # Initialize the generator node for 100Hz udpates, using the above
    # Trajectory class.
    generator = GeneratorNode('generator', 100, Trajectory)

    # Spin, meaning keep running (taking care of the timer callbacks
    # and message passing), until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
