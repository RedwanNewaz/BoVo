#!/usr/bin/python3
import rospy 
from geometry_msgs.msg import PoseWithCovarianceStamped 
from nav_msgs.msg import Odometry
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from time import sleep 
import numpy as np


class AprilTagConverter:
    def __init__(self) -> None:
        self._state_sub = rospy.Subscriber('tag_detections', AprilTagDetectionArray, self.callback_apriltag)
        # self._odom_sub = rospy.Subscriber('/roomba20/odom/', Odometry, self.callabcak_odom)
        # self._pub_odom = rospy.Publisher('/roomba20/odom/fix', Odometry, queue_size=10)
        self._pose_pubs = {tagID: rospy.Publisher('/roomba%s/apriltag' % tagID, PoseWithCovarianceStamped, queue_size=10) for tagID in [20, 21]}


    def filtered_msg(self, group_id, poseMsg):
        rospy.loginfo(f"pose = {poseMsg} \n id = {group_id}") 
        poseMsg.header.frame_id = "camera"
        poseMsg.pose.pose.position.y *= 1.315
        self._pose_pubs[group_id].publish(poseMsg) 

    def callback_apriltag(self, msg):
        for pose in msg.detections:
            self.filtered_msg(pose.id[0], pose.pose)

    def callabcak_odom(self, msg:Odometry):
        msg.pose.position.y *= -1 
        self._pub_odom.publish(msg)

            


if __name__ =="__main__":
    rospy.init_node("apriltag_converter", anonymous=True)
    rospy.loginfo("[+] apriltag converted node started !!")
    tag = AprilTagConverter()
    rospy.spin()
