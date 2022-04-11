#!/usr/bin/python3
# https://github.com/MengGuo/RVO_Py_MAS/blob/master/example.py
from RVO import RVO_update, reach, compute_V_des, reach, distance
from geometry_msgs.msg import Point, PoseStamped
import rospy
from time import sleep

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


def publish_setpoint(setpoint, pub):
    pose = PoseStamped()
    pose.pose.position.x = setpoint[0]
    pose.pose.position.y = setpoint[1]
    pub.publish(pose)


if __name__ == '__main__':
    rospy.init_node('roomba_traj_viewer', anonymous=True)

    # robot radius
    ws_model['robot_radius'] = rospy.get_param("/%s/radius" % rospy.get_name(), 0.345)
    robot1 = rospy.get_param("/%s/robot20" % rospy.get_name())
    robot2 = rospy.get_param("/%s/robot21" % rospy.get_name())
    goal_topics = rospy.get_param("/%s/goal_topics" % rospy.get_name())
    goal_pubs = [rospy.Publisher(topic, PoseStamped, queue_size=10) for topic in goal_topics]

    X = [robot1.pop(0), robot2.pop(0)]
    # velocity of [vx,vy]
    V = [[0, 0] for i in range(len(X))]
    # maximal velocity norm
    V_max = [1.0 for i in range(len(X))]
    # goal of [x,y]
    goal = [robot1[0], robot2[0]]
    rospy.loginfo(f"initial goal = {goal}")
    sleep(5)  # take a nap
    step = 0.5
    rate = rospy.Rate(step)
    while robot1 or robot2:
        # compute desired vel to goal
        V_des = compute_V_des(X, goal, V_max)
        # compute the optimal vel to avoid collision
        V = RVO_update(X, V_des, V, ws_model)
        # update position
        for i in range(len(X)):
            X[i][0] += V[i][0] * step
            X[i][1] += V[i][1] * step
            # check i th robot reach the goal location or not
            if ws_model['robot_radius'] > distance(X[i], goal[i]):
                rospy.loginfo(f"goal{i} = {goal[i]}")
                if i == 0 and robot1:
                    goal[i] = robot1.pop(0)
                elif i == 1 and robot2:
                    goal[i] = robot2.pop(0)
            # print(len(robot1) + len(robot2))
            publish_setpoint(X[i], goal_pubs[i])
        rate.sleep()
    print("[+] terminated")
    sleep(5)
