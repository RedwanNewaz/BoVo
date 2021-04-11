import unittest
import numpy as np
import json
from vo import RobotModel

obstacles = [(-24,0), (-5.40, -1.67), (-8.67, -0.60), (-11.83, -0.26)]

class CollisionCheck(unittest.TestCase):
    def setUp(self) -> None:
        start = np.array([-2.6, -10.6, 0, 0])
        goal = np.array([-12.6, -6, 0, 0])
        r = RobotModel(start, goal, 1)
        self.workspace = r.road_map.workspace

    def test_collision(self):
        for x, y in obstacles:
            self.assertTrue(self.workspace.check_collision(x, y))

    def test_valid_trajectories(self):
        with open('valid_traj.json') as jfile:
            traj = json.load(jfile)
        for key, data in traj.items():
            for x, y in data:
                self.assertFalse(self.workspace.check_collision(x, y), '({:.3f} {:.3f})'.format(x, y))


    def test_trajectories(self):
        with open('traj.json') as jfile:
            traj = json.load(jfile)
        for key, data in traj.items():
            for x, y in data:
                self.assertFalse(self.workspace.check_collision(x, y), '({:.3f} {:.3f})'.format(x, y))



if __name__ == '__main__':
    unittest.main(verbosity=True)