from matplotlib.patches import Rectangle
import matplotlib.pyplot as plt
import json
import fcl
import numpy as np
class RectangleObstacle:
    def __init__(self, origin, width, height, offset):
        self.origin = origin
        self.width = width
        self.height = height
        self.offset = offset
        self.center = ((origin[0]+width/2), (origin[1]+height/2), 0)
        self.create_fcl_objs(offset)
    def create_fcl_objs(self, offset):
        self.g1 = fcl.Box(self.width, self.height, 3)
        self.t1 = fcl.Transform(np.array(self.center))
        self.o1 = fcl.CollisionObject(self.g1, self.t1)
        self.g2 = fcl.Sphere(offset)

    def plot(self):
        self.offset = 0
        xy = ((self.origin[0]-self.offset), (self.origin[1]-self.offset))
        width = self.width + self.offset
        height = self.height + self.offset
        color = 'gray'
        return Rectangle(xy, width=width, height=height, alpha = 0.7, fc=color,ec=color,)
    def coords(self, addOffset=True):
        offset = 0
        if addOffset:
            offset = self.offset
        xy = ((self.origin[0]-offset), (self.origin[1]-offset))
        w = self.width + offset
        h = self.height + offset
        return [xy, (xy[0]+w, xy[1]), (xy[0]+w, xy[1]+h), (xy[0], xy[1]+h)]

    def check_point_in(self, x, y):
        check_x = x>=self.origin[0] and x<= self.origin[0] + self.width
        check_y = y>=self.origin[1] and y<= self.origin[1] + self.height
        return check_x and check_y

    def collision_check(self, x, y):
        t2 = fcl.Transform(np.array([x, y, 0]))
        o2 = fcl.CollisionObject(self.g2, t2)
        request = fcl.ContinuousCollisionRequest()
        result = fcl.ContinuousCollisionResult()
        ret = fcl.continuousCollide(self.o1, self.t1, o2, t2, request, result)
        return ret == 0.0


class Obstacles:
    def __init__(self, name, robot_radius = 1.0):
        with open(name) as file:
            self.env = json.load(file)
        self.rect_obstacles = []
        self.name = name
        for key, data in self.env.items():
            # print(key, data)
            position = data['position']
            shape = data['shape']

            width = shape[0]
            height = shape[1]
            xy = (position[0] - width / 2, position[1] - height / 2)
            self.rect_obstacles.append(RectangleObstacle(xy, width, height, robot_radius))

    def check_collision(self, pos):
        x, y = pos[0], pos[1]
        collisions = [r.collision_check(x, y)for r in self.rect_obstacles]
        return any(collisions)
    def plot(self):
        ax = plt.gca()
        for rect in self.rect_obstacles:
            ax.add_patch(rect.plot())

    def tolist(self):
        return [item .coords(addOffset=False) for item in self.rect_obstacles]


    def copy(self):
        return Obstacles(self.name)