from .velocity_obstacle import compute_desired_velocity, compute_velocity, update_state
import numpy as np
from visibility_roadmap import VisibilityPlanner
import os
ROBOT_RADIUS =1
VMAX = 5
filedir = os.path.dirname(__file__)

class RobotModel:
    def __init__(self, robot_start, robot_goal, id):
        self.id = id
        self.road_map = VisibilityPlanner(os.path.join(filedir,'..', 'visibility_roadmap/map2.json'))
        path = self.road_map(robot_start[:2], robot_goal[:2])
        self.add_waypoints(path)
    def __call__(self, others):
        v_desired = compute_desired_velocity(self.state, self.goal, ROBOT_RADIUS, VMAX)
        control_vel = compute_velocity(self.state, others, v_desired, ROBOT_RADIUS, VMAX, self.road_map.workspace)
        self.state = update_state(self.state, control_vel)


    def add_waypoints(self, path):
        self.path = path
        self.count = 0
        terminal_vel = np.array((0, 0))
        self.goal = np.hstack((self.path[self.count], terminal_vel))
        self.state = np.hstack((self.path[self.count], terminal_vel))

    def update(self):
        if self.isArrived():
            if self.count < len(self.path)-1:
                self.count += 1
                terminal_vel = np.array((0,0))
                self.goal = np.hstack((self.path[self.count], terminal_vel))
                print('[+] robot {} | goal {}'.format(self.id, self.goal))
            else:
                self.count += 1
        status = self.count < len(self.path)
        return status


    def isArrived(self):
        if np.linalg.norm(self.state[:2] - self.goal[:2]) < 0.40:
            return True
        return False

    def obstacles(self, others):
        for o in others:
            if not (o == self):
                yield o

    def __eq__(self, other):
        return other.id == self.id

    def get_position(self):
        # print(self.id, self.state)
        return self.state[:2]