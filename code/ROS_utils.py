"""Utilities for ROS allowing abstraction of the publishers for the demo
   in order to simplify and neaten the demo code."""

import time

from code.TransformHelpers      import Transform_from_T, Point_from_p
from tf2_ros                    import TransformBroadcaster
from sensor_msgs.msg            import JointState
from geometry_msgs.msg          import TransformStamped
from rclpy.qos                  import QoSProfile, DurabilityPolicy
from geometry_msgs.msg          import Point, Vector3, Quaternion
from std_msgs.msg               import ColorRGBA
from visualization_msgs.msg     import Marker, MarkerArray

# Robot state publisher object
class robo_publisher():
    #Initialization
    def __init__(self, node, joint_list):
        self.node = node

        # Init transform broadcaster
        self.broadcaster = TransformBroadcaster(self.node)

        # Save the joint names
        self.joint_list = joint_list

        # Add a publisher to send the joint states
        self.pub = self.node.create_publisher(JointState, '/joint_states', 10)

        # Wait for a connection to happen
        self.node.get_logger().info("Waiting for a /joint_states subscriber...")
        while self.pub.get_subscription_count() == 0:
            time.sleep(0.1)

    def publish(self, Tpelvis, q, qdot):
        """Given the tranform matrix for the pelvis from the world, and the q
        and qdot vectors for all the joints, publishe the joint states and
        transform."""
        now = self.node.now().to_msg()

        # Build up and send the pelvis w.r.t the world
        trans = TransformStamped()
        trans.header.stamp = now
        trans.header.frame_id = "world"
        trans.child_frame_id = "pelvis"
        trans.transform = Transform_from_T(Tpelvis)
        self.broadcaster.sendTransform(trans)

        # Build up and publish the joint states
        cmdmsg = JointState()
        cmdmsg.header.stamp = now
        cmdmsg.name = self.joint_list
        cmdmsg.position = q
        cmdmsg.velocity = qdot
        self.pub.publish(cmdmsg)


# Ball marker array publisher
class ball_publisher():
    # Initialization
    def __init__(self, node, radius):
        self.node = node
        self.radius = radius

        # Prepare the publisher
        quality = QoSProfile(durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             depth=1)
        self.pub = self.node.create_publisher(MarkerArray,
                                              '/visualization_marker_array',
                                              quality)
        
        # Create the sphere marker
        diam = 2 * radius
        self.marker = Marker()
        self.marker.header.frame_id  = "world"
        self.marker.header.stamp     = self.node.get_clock().now().to_msg()
        self.marker.action           = Marker.ADD
        self.marker.ns               = "point"
        self.marker.id               = 1
        self.marker.type             = Marker.SPHERE
        self.marker.pose.orientation = Quaternion()
        self.marker.pose.position    = Point_from_p(self.node.traj.p0ball)
        self.marker.scale            = Vector3(x = diam, y = diam, z = diam)
        self.marker.color            = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.8)

        # Create the marker array message.
        self.mark = MarkerArray()
        self.mark.markers.append(self.marker)

    def publish(self, pball):
        """Given the current position of the ball, publish the ball marker."""
        self.marker.header.stamp = self.node.now().to_msg()
        self.marker.pose.position = Point_from_p(pball)
        self.pub.publish(self.mark)
