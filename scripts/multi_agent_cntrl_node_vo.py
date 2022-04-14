#!/usr/bin/python3
# https://github.com/MengGuo/RVO_Py_MAS/blob/master/example.py
from RVO import RVO_update, reach, compute_V_des, reach, distance
from geometry_msgs.msg import Point, PoseStamped
import rospy
from time import sleep
from threading import Thread
from nav_msgs.msg import Odometry
from traj_view.srv import measurement

class MultiAgentSystem(Thread):
    dt = 0.3
    # ------------------------------
    # define workspace model
    ws_model = dict()

    # circular obstacles, format [x,y,rad]
    # no obstacles
    ws_model['circular_obstacles'] = []
    # with obstacles
    # ws_model['circular_obstacles'] = [[-0.3, 2.5, 0.3], [1.5, 2.5, 0.3], [3.3, 2.5, 0.3], [5.1, 2.5, 0.3]]
    # rectangular boundary, format [x,y,width/2,heigth/2]
    ws_model['boundary'] = []

    def __init__(self):
        super().__init__()
        self.ws_model['robot_radius'] = rospy.get_param("/%s/radius" % rospy.get_name(), 0.345)
        self.__paths = [rospy.get_param("/%s/%s" % (rospy.get_name(), path)) for path in ['robot20', 'robot21']]
        self.__cntrl_pubs = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in rospy.get_param("/%s/goal_topics" % rospy.get_name())]

    @staticmethod
    def publish_setpoint(setpoint, pub):
        pose = PoseStamped()
        pose.pose.position.x = setpoint[0]
        pose.pose.position.y = setpoint[1]
        pub.publish(pose)

    def get_measurement(self, X):
        msg = Odometry()
        msg.pose.pose.position.x = X[0]
        msg.pose.pose.position.y = X[1]
        try:
            # create a handle to the add_two_ints service
            sensor_reading = rospy.ServiceProxy('sensor_reading', measurement)
            z = sensor_reading(msg)
            print(z)
        except rospy.ServiceException as e:
            print("Service call failed: %s" % e)


    def run(self):
        X = [path.pop(0) for path in self.__paths]
        # velocity of [vx,vy]
        V = [[0, 0] for _ in range(len(X))]
        # maximal velocity norm
        V_max = [1.0 for _ in range(len(X))]
        # goal of [x,y]
        goal = X.copy()
        # delay function
        rate = rospy.Rate(self.dt)

        while not rospy.is_shutdown():
            # compute desired vel to goal
            V_des = compute_V_des(X, goal, V_max)
            # compute the optimal vel to avoid collision
            V = RVO_update(X, V_des, V, self.ws_model)

            # update position
            for i in range(len(X)):
                X[i][0] += V[i][0] * self.dt
                X[i][1] += V[i][1] * self.dt
                # check i th robot reach the goal location or not
                if self.ws_model['robot_radius'] > distance(X[i], goal[i]):
                    if self.__paths[i]:
                        goal[i] = self.__paths[i].pop(0)
                        rospy.loginfo(f"goal{i} = {goal[i]}")
                self.publish_setpoint(X[i], self.__cntrl_pubs[i])
                self.get_measurement(X[i])
            rate.sleep()
        print("[+] terminated")
        sleep(5)


if __name__ == '__main__':
    rospy.init_node('roomba_traj_viewer', anonymous=True)
    mas = MultiAgentSystem()
    print('waiting for sensor_reading')
    rospy.wait_for_service('sensor_reading')
    print('node started')
    mas.start()
    mas.join()
