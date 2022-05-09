#!/usr/bin/python3
import rospy
from geometry_msgs.msg import Vector3Stamped, PoseStamped
from AsyncBO import opt_acquisition
from sklearn.gaussian_process import GaussianProcessRegressor
import numpy as np
class FieldSampler:
    def __init__(self):
        self.__stf_sub = rospy.Subscriber("/stf/data", Vector3Stamped, self.stf_field_callback)
        self.goalTopics = ["roomba20/goal", "roomba21/goal"]
        self.__goal_pub = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in self.goalTopics]
        self.__data = list()
        self.__timer = rospy.Timer(rospy.Duration(5), self.timer_callback)
        self.__robots = {}
        self.__lastN = 0
        self.field_predictor = GaussianProcessRegressor()
    def stf_field_callback(self, msg):
        sample = (msg.vector.x, msg.vector.y, msg.vector.z)
        if sample not in self.__data:
            self.__data.append(sample)
        # print(self.__data)
        self.__robots[msg.header.frame_id] = (msg.vector.x, msg.vector.y)
        if len(self.__data) > 1000:
            self.__data.pop(0)
    def timer_callback(self, event):
        if len(self.__data) <= 0: return
        # if len(self.__data) <= self.__lastN: return
        self.__lastN = len(self.__data)
        X = np.array([[x[0], x[1]] for x in self.__data])
        Z = np.array([x[2] for x in self.__data])
        self.field_predictor.fit(X, Z)
        rospy.loginfo(f"training data size {self.__lastN}")
        for i, pos in enumerate(self.__robots.values()):
            nextBest = opt_acquisition(pos, X, self.field_predictor)
            # print(i, nextBest)
            self.publish_goal(i, nextBest)
    def publish_goal(self, robotID, goal):
        msg = PoseStamped()
        msg.pose.position.x = goal[0]
        msg.pose.position.y = goal[1]
        self.__goal_pub[robotID].publish(msg)


if __name__ == '__main__':
    rospy.init_node("bovo_sampler", anonymous=True)
    rospy.loginfo("bovo sampler initialized")
    fs = FieldSampler()
    rospy.spin()