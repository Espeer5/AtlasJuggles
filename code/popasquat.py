# Forces Atlas to squat down and stand back up

import rclpy
import numpy as np

from math import pi, sin, cos

# Import the format for the condition number message
from std_msgs.msg import Float64

# Grab the utilities
from hw5code.TransformHelpers   import *
from hw5code.TrajectoryUtils    import *
from code.TransformHelpers     import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState

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
        self.lchain = KinematicChain(node, 'pelvis', 'l_foot', self.ljointnames())
        self.rchain = KinematicChain(node, 'pelvis', 'r_foot', self.rjointnames())

        # Initialize starting position
        init_leg = np.radians(np.radians(np.array([[0.0], [0.0], [-15], [15], [0.0], [0.0]])))
        self.q0 = np.concatenate((init_leg, init_leg), axis=0)
        self.pl0 = self.lchain.fkin(self.q0[:6])[0]
        self.pr0 = self.rchain.fkin(self.q0[6:])[0]

        # Initialize trajectory variables
        self.q = self.q0 # current joint angles
        self.lamb = 5 # lambda for ikin algorithm
        self.T = 5 # Period of squatting motion
        self.lambS = 10 # lambda for secondary task

    def ljointnames(self):
        # Return the left leg jointnames
        return ['l_leg_hpz', 'l_leg_hpx', 'l_leg_hpy', 'l_leg_kny', 'l_leg_aky',
                'l_leg_akx']
    
    def rjointnames(self):
        # Return the right leg jointnames
        return ['r_leg_hpz', 'r_leg_hpx', 'r_leg_hpy', 'r_leg_kny', 'r_leg_aky',
                'r_leg_akx']
    
    def jointnames(self):
        # Return the jointnames
        return self.ljointnames() + self.rjointnames()
    
    def evaluate(self, t, dt):
        if t < 0.5:
            p, v = goto(t, 0.5, 0, 1)
            self.q[2] = np.radians(-15 * p)
            self.q[8] = np.radians(-15 * p)
            self.q[3] = np.radians(15 * p)
            self.q[9] = np.radians(15 * p)
            qdot = np.zeros((len(self.q), 1))
            qdot[2] = np.radians(-15 * v)
            qdot[8] = np.radians(-15 * v)
            qdot[3] = np.radians(15 * v)
            qdot[9] = np.radians(15 * v)
            return (self.q.flatten().tolist(), qdot.flatten().tolist())
        else:
            t=t-0.5
        pdl = self.pl0 + np.array([0.0, 0.0, 0.1 + 0.08 * cos(2 * pi / self.T * t)]).reshape(3, 1)
        pdr = self.pr0 + np.array([0.0, 0.0, 0.1 + 0.08 * cos(2 * pi / self.T * t)]).reshape(3, 1)
        Rd = Reye()
        vd = np.array([0, 0, -0.1 * 2 * pi / self.T * sin(2 * pi / self.T * t)]).reshape(3, 1)
        wd = np.array([0, 0, 0]).reshape(3, 1)

        # Compute the old fkin
        pl, Rl, Jvl, Jwl = self.lchain.fkin(self.q[:6])
        pr, Rr, Jvr, Jwr = self.rchain.fkin(self.q[6:])

        epdl = ep(pdl, pl)
        epdr = ep(pdr, pr)
        eRl = eR(Rd, Rl)
        eRr = eR(Rd, Rr)
    
        # Form the Jacs
        Jl = np.vstack((Jvl, Jwl))
        Jr = np.vstack((Jvr, Jwr))

        # Form the 6d errors
        el = np.vstack((epdl, eRl))
        er = np.vstack((epdr, eRr))

        # Form the desired x_dots
        xd = np.vstack((vd, wd))

        # Compute the new joint angles
        qdotl = np.linalg.pinv(Jl) @ (xd + self.lamb * el)
        qdotr = np.linalg.pinv(Jr) @ (xd + self.lamb * er)

        qdot = np.concatenate((qdotl, qdotr), axis=0)

        q = self.q + qdot * dt

        self.q = q

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)
        self.traj = Trajectory(self)

    # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Add a publisher to send the joint commands.
        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # Set up the timing so (t=0) will occur in the first update
        # cycle (dt) from now.
        self.dt    = 1.0 / float(rate)
        self.t     = -self.dt
        self.start = self.get_clock().now() + Duration(seconds=self.dt)

        # Create a timer to keep calling update().
        self.create_timer(self.dt, self.update)
        self.get_logger().info("Running with dt of %f seconds (%fHz)" %
                               (self.dt, rate))

    # Shutdown.
    def shutdown(self):
        # Destroy the node, including cleaning up the timer.
        self.destroy_node()

    # Return the current time (in ROS format).
    def now(self):
        return self.start + Duration(seconds=self.t)

    # Update - send a new joint command every time step.
    def update(self):
        # To avoid any time jitter enforce a constant time step and
        # integrate to get the current time.
        self.t += self.dt

        # Evaluate the trajectory at the current time.
        (q_r, qdot_r) = self.traj.evaluate(self.t, self.dt)

        # Compute position/orientation of the pelvis (w.r.t. world).
        ppelvis = pxyz(0.0, 0.0, 0.5)
        Rpelvis = Rotz(0)
        Tpelvis = T_from_Rp(Rpelvis, ppelvis)
        
        # Build up and send the Pelvis w.r.t. World Transform!
        trans = TransformStamped()
        trans.header.stamp    = self.now().to_msg()
        trans.header.frame_id = 'world'
        trans.child_frame_id  = 'pelvis'
        trans.transform       = Transform_from_T(Tpelvis)
        self.broadcaster.sendTransform(trans)

        q = [0.0 for _ in range(len(jointnames))]
        qdot = [0.0 for _ in range(len(jointnames))]

        for joint_name, curr_q, curr_qdot in zip(self.traj.jointnames(), q_r, qdot_r):
            index = jointnames.index(joint_name)
            q[index] = curr_q
            qdot[index] = curr_qdot

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = q                         # List of positions
        cmdmsg.velocity     = qdot                      # List of velocities
        self.pub.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    # generator = GeneratorNode('generator', 100, Trajectory)
    node = DemoNode('popasquat', 100)

    # Spin, until done
    # generator.spin()
    rclpy.spin(node)

    # Shutdown the node and ROS.
    # generator.shutdown() 
    node.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
