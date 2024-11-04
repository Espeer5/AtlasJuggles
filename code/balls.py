# Forces Atlas to squat down and stand back up

import rclpy
import numpy as np

from math import pi, sin, cos, sqrt, acos

# Import the format for the condition number message
from std_msgs.msg import Float64

# Grab the utilities
from hw5code.TransformHelpers   import *
from code.TrajectoryUtils    import *
from code.TransformHelpers     import *

# Grab the general fkin from HW5 P5.
from hw5code.KinematicChain     import KinematicChain

from rclpy.node                 import Node
from rclpy.time                 import Duration
from tf2_ros                    import TransformBroadcaster
from geometry_msgs.msg          import TransformStamped
from sensor_msgs.msg            import JointState
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker
from visualization_msgs.msg     import MarkerArray

# Constants 
GRAV = -1

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
        self.headchain = KinematicChain(node, 'pelvis', 'head', self.headjointnames())

        # Initialize starting position
        init_leg = np.radians(np.array([[0.0], [0.0], [-15], [15], [0.0], [0.0]]))
        q0head = np.radians(np.array([[0.0], [0.0], [0.0], [0.0]]))
        self.q0 = np.concatenate((init_leg, init_leg, q0head), axis=0)
        self.pl0 = self.lchain.fkin(self.q0[:6])[0]
        self.pr0 = self.rchain.fkin(self.q0[6:12])[0]

        self.p_pelvis = pxyz(0.0, 0.0, 0.862)

        self.p0ball = np.array([[0.1], [-0.08], [5]]).reshape(3, 1) + self.p_pelvis
        self.pball = np.copy(self.p0ball)
        self.ball_radius = 0.1
        self.v0ball = np.array([0.0, 0.0, 0.0]).reshape((3,1))
        self.vball = np.copy(self.v0ball)
        self.a0ball = np.array([0.0, 0.0, GRAV]).reshape((3,1))
        self.aball = np.copy(self.a0ball)

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
    
    def headjointnames(self):
        return ['back_bkz', 'back_bky', 'back_bkx', 'neck_ry']
    
    def jointnames(self):
        # Return the jointnames
        return self.ljointnames() + self.rjointnames() + self.headjointnames()
    
    def evaluate(self, t, dt):
        
        ph_goal = np.array([[0.1], [-0.08], [.8]])
        T = sqrt((2*(self.p0ball[2, 0] + 1.6955))/-GRAV)
        hpath, hpath_v = goto(t, T, 0, 1)
        pdh = self.p0ball + hpath * (ph_goal - self.p0ball)
        vdh = hpath_v * (ph_goal - self.p0ball)

        pdl = self.pl0
        pdr = self.pr0
        Rd = Reye()
        vd = np.zeros((3,1))
        wd = np.zeros((3,1))

        qhead = self.q[12:]
    
        # Compute the old fkin
        phead, Rhead, Jvhead, Jwhead = self.headchain.fkin(qhead)
        pl, Rl, Jvl, Jwl = self.lchain.fkin(self.q[:6])
        pr, Rr, Jvr, Jwr = self.rchain.fkin(self.q[6:12])

        vx_bounce, vy_bounce, vz_bounce = calculate_kicking_velocity(ph_goal, ph_goal, T)

        v_bounce = np.array([[vx_bounce],[vy_bounce],[vz_bounce]]).reshape(3,)
        v_unit = v_bounce/np.linalg.norm(v_bounce)
        z_i = np.array([[0.0], [0.0], [1.0]]).reshape(3,)
        e_axis_head = z_i * v_unit
        alpha = acos(np.dot(z_i, v_unit)/(np.linalg.norm(z_i)*np.linalg.norm(v_unit)))
        alphadot = hpath_v * alpha
        omega = e_axis_head * alphadot
        R_goal = Rote(e_axis_head, alpha)
        Rdh = Rote(e_axis_head, alpha * hpath)

        eph = ep(pdh, phead)
        epdl = ep(pdl, pl)
        epdr = ep(pdr, pr)
        eRl = eR(Rd, Rl)
        eRr = eR(Rd, Rr)
        eRh = eR(Rdh, Rhead)
        
        # Form the Jacs
        Jl = np.vstack((Jvl, Jwl))
        Jr = np.vstack((Jvr, Jwr))
        Jh = np.vstack((Jvhead, Jwhead))

        # Form the 6d errors
        el = np.vstack((epdl, eRl))
        er = np.vstack((epdr, eRr))
        eh = np.vstack((eph, eRh))

        # Form the desired x_dots
        xd = np.vstack((vd, wd))
        xdh = np.vstack((vdh.reshape(3,1), omega.reshape(3,1)))

        # Compute the new joint angles
        qdotl = np.linalg.pinv(Jl) @ (xd + self.lamb * el)
        qdotr = np.linalg.pinv(Jr) @ (xd + self.lamb * er)
        qdoth = np.linalg.pinv(Jh) @ (xdh + self.lamb * eh)

        qdot = np.concatenate((qdotl, qdotr, qdoth), axis=0)

        q = self.q + qdot * dt

        self.q = q

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())

    def evaluate_ball(self, t, dt):
        # Evaluate the trajectory at the current time.
        # print(self.pball, self.vball)
        
        
        # find position and orientation of head
        phead, R, _, _ = self.headchain.fkin(self.q[12:])
        

        # must be in this order
        self.vball += dt * self.aball
        self.pball += dt * self.vball

        # check to see if we are colliding with head
        
        print(self.pball, phead)
        print("---")
        if np.linalg.norm(self.pball - phead - self.p_pelvis) < self.ball_radius:
            # find angle to bounce off
            # theta_bounce = 
            self.pball[2,0] += self.ball_radius
            # self.vball[2,0] *= -1.0
            # setting surface normal to z axis for now
            norm = np.array([0, 0, 1]).reshape(3, 1) 
            self.vball = self.vball - 2 * (norm * self.vball) * norm
            # update stuff

            # move out later :
            # find the velocity of the next bounce

            
        # set velocity and position
        # print(self.pball, self.vball)
        # print("--- --- ---")
        return self.pball, self.vball


#
#   Demo Node Class
#
class DemoNode(Node):
    # Initialization
    def __init__(self, name, rate):
        # Initialize the node, naming it as specified
        super().__init__(name)
        self.traj = Trajectory(self)

    # ROBOT STUFF

    # Initialize the transform broadcaster
        self.broadcaster = TransformBroadcaster(self)

        # Add a publisher to send the joint commands.
        self.pub_j = self.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen.  This isn't necessary, but
        # means we don't start until the rest of the system is ready.
        self.get_logger().info("Waiting for a /joint_states subscriber...")
        while(not self.count_subscribers('/joint_states')):
            pass

        # BALL STUFF
        # Prepare the publisher (latching for new subscribers).
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub_c = self.create_publisher(MarkerArray, '/visualization_marker_array', quality)

        # Initialize the ball position, velocity, set the acceleration.
        self.radius = 0.1

        # self.p = np.array([0.0, 0.0, self.radius + 1]).reshape((3,1))
        # self.v = np.array([0.0, 0.0,  5.0       ]).reshape((3,1))
        # self.a = np.array([0.0, 0.0, GRAV      ]).reshape((3,1))

        # Create the sphere marker.
        diam        = 2 * self.radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.traj.p0ball)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)
        # a = 0.8 is slightly transparent!

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)

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

        # ROBOT STUFF

        # Evaluate the trajectory at the current time.
        (q_r, qdot_r) = self.traj.evaluate(self.t, self.dt)

        # Compute position/orientation of the pelvis (w.r.t. world).
        ppelvis = pxyz(0.0, 0.0, 0.862)
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

        # BALL STUFF

        # Integrate the velocity, then the position.
        # self.v += self.dt * self.a
        # self.p += self.dt * self.v

        # Check for a bounce - not the change in x velocity is non-physical.
        # if self.p[2,0] < self.radius + 1:
        #     self.p[2,0] = self.radius + 1 + (self.radius - self.p[2,0])
        #     self.v[2,0] *= -1.0
        pball, _ = self.traj.evaluate_ball(self.t, self.dt)

        self.marker.header.stamp  = self.now().to_msg()
        self.marker.pose.position = Point_from_p(pball)
        self.pub_c.publish(self.mark)

        # Build up a command message and publish.
        cmdmsg = JointState()
        cmdmsg.header.stamp = self.now().to_msg()       # Current time for ROS
        cmdmsg.name         = jointnames                # List of names
        cmdmsg.position     = q                         # List of positions
        cmdmsg.velocity     = qdot                      # List of velocities
        self.pub_j.publish(cmdmsg)


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the demo node (100Hz).
    rclpy.init(args=args)
    # generator = GeneratorNode('generator', 100, Trajectory)
    node = DemoNode('balls', 100)

    # Spin, until done
    # generator.spin()
    rclpy.spin(node)

    # Shutdown the node and ROS.
    # generator.shutdown() 
    node.shutdown()

    rclpy.shutdown()

if __name__ == "__main__":
    main()
