#!/usr/bin/python3
import ros
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped, TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg import Marker

np.set_printoptions(precision=4)


class TrajController:
    dt = 0.03

    def __init__(self, Kp_rho, Kp_alpha, Kp_beta):
        init_pos = rospy.get_param('/%s/init_pos' % rospy.get_name(), [0, 0, 0])
        self._target_state = np.array(init_pos)
        self._curr_state = np.array(init_pos, dtype=float)
        self.Kp_rho = Kp_rho
        self.Kp_alpha = Kp_alpha
        self.Kp_beta = Kp_beta
        name = rospy.get_param('/%s/name' % rospy.get_name(), "roomba20")
        self._path_pub = rospy.Publisher('/%s/path' % name, Marker, queue_size=10)
        self._cmd_pub = rospy.Publisher('/%s/cmd_vel' % name, TwistWithCovarianceStamped, queue_size=10)
        self._odom_pub = rospy.Publisher('/%s/odom' % name, Odometry, queue_size=10)

        self._state_sub = rospy.Subscriber('/%s/%s/filtered/odometry' % (name, name), Odometry, self.state_callback)

    def state_callback(self, msg:Odometry):
        pose = msg.pose.pose
        self._curr_state[0] = pose.position.x
        self._curr_state[1] = pose.position.y
        eulers = euler_from_quaternion([pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w])
        self._curr_state[2] = eulers[-1]


    def goal_update(self, msg: PoseStamped):
        """
        trajectory view node publish goal position that will be subscribed here
        :param msg: PoseStamp message
        :return: update internal variable target state
        """
        x = msg.pose.position.x
        y = msg.pose.position.y

        dx = x - self._curr_state[0]
        dy = y - self._curr_state[1]
        theta = np.arctan2(dy, dx)
        self._target_state = np.array([x, y, theta], dtype=float)

    def calc_control_command(self, x_diff, y_diff, theta, theta_goal):
        """
        Returns the control command for the linear and angular velocities as
        well as the distance to goal
        Parameters
        ----------
        x_diff : The position of target with respect to current robot position
                 in x direction
        y_diff : The position of target with respect to current robot position
                 in y direction
        theta : The current heading angle of robot with respect to x axis
        theta_goal: The target angle of robot with respect to x axis
        Returns
        -------
        rho : The distance between the robot and the goal position
        v : Command linear velocity
        w : Command angular velocity
        """

        # Description of local variables:
        # - alpha is the angle to the goal relative to the heading of the robot
        # - beta is the angle between the robot's position and the goal
        #   position plus the goal angle
        # - Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards
        #   the goal
        # - Kp_beta*beta rotates the line so that it is parallel to the goal
        #   angle
        #
        # Note:
        # we restrict alpha and beta (angle differences) to the range
        # [-pi, pi] to prevent unstable behavior e.g. difference going
        # from 0 rad to 2*pi rad with slight turn

        rho = np.hypot(x_diff, y_diff)
        alpha = (np.arctan2(y_diff, x_diff)
                 - theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi
        v = self.Kp_rho * rho
        w = self.Kp_alpha * alpha - self.Kp_beta * beta

        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v

        return rho, v, w

    @staticmethod
    def current_state_pub(state):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = rospy.Time.now()
        name = rospy.get_param('/%s/name' % rospy.get_name(), "roomba20")
        marker.ns = '/%s/controlState' % name
        marker.id = 202
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.type = Marker.MESH_RESOURCE
        # marker.mesh_resource = "file:///home/redwan/ros_catkin_place/catkin_roomba/src/irobotcreate2ros/iRobot/iRobot2.dae"
        marker.mesh_resource = "package://traj_view/config/iRobot2.dae"
        # set up scale of the points
        marker.scale.x = 1
        marker.scale.y = 1
        marker.scale.z = 1

        marker.color.a = 0.80
        marker.color.r = 0.663
        marker.color.g = 0.663
        marker.color.b = 0.663

        marker.pose.position.x = state[0]
        marker.pose.position.y = state[1]
        marker.pose.position.z = 0.0

        q = quaternion_from_euler(0, 0, state[-1])
        marker.pose.orientation.x = q[0]
        marker.pose.orientation.y = q[1]
        marker.pose.orientation.z = q[2]
        marker.pose.orientation.w = q[3]

        return marker

    def cntrl_update(self, event):
        """
        :param event: timer event
        :return:compute control parameters
        """
        state_diff = self._target_state - self._curr_state
        rho, v, w = self.calc_control_command(state_diff[0], state_diff[1], self._curr_state[-1],
                                              self._target_state[-1])

        if rho <= 0.1:
            v, w = 0, 0

        msg = TwistWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'

        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w
        # msg.linear.x = v
        # msg.angular.z = w
        self._cmd_pub.publish(msg)

        curr_state = self._curr_state.copy()

        # TODO replace this with ekf localiazation
        curr_state[2] += w * self.dt
        theta = curr_state[2]
        curr_state[0] += v * np.cos(theta) * self.dt
        curr_state[1] += v * np.sin(theta) * self.dt
        # print(curr_state)
        marker = self.current_state_pub(curr_state)
        self._path_pub.publish(marker)

        # odometry publish
        odom = Odometry()
        odom.header.frame_id = 'map'
        odom.child_frame_id = 'odom'
        odom.header.stamp = rospy.Time.now()
        odom.pose.pose.position.x = curr_state[0]
        odom.pose.pose.position.y = curr_state[1]

        q = quaternion_from_euler(0, 0, curr_state[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        self._odom_pub.publish(odom)

if __name__ == "__main__":
    rospy.init_node("TrajController", anonymous=True)
    rospy.loginfo("Traj Controller Initiated")
    trajCntrl = TrajController(0.6, 1, 0.2)
    rate_cntrl_update = 30.0
    action_topic = rospy.get_param('/%s/action_topic' % rospy.get_name(), "/move_base_simple/goal")
    rospy.Subscriber("%s" % action_topic, PoseStamped, trajCntrl.goal_update)
    rospy.Timer(rospy.Duration(1.0 / rate_cntrl_update), trajCntrl.cntrl_update)
    rospy.spin()
