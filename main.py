import matplotlib.pyplot as plt
import numpy as np
from vo import RobotModel
from collections import defaultdict
# from vrep_sim import vrep_trajectories
from matplotlib.patches import Circle
import json

def gen_trajectories(robots):
    while any([r.update() for r in robots]):
        for r in robots:
            others = r.obstacles(robots)
            if r.update():
                r(others)
                yield {r.id:r.get_position()}
    print('[+] terminated')
if __name__ == '__main__':
    # robot1_start = np.array([-2.6, -10.6, 0, 0])
    robot1_start = np.array([7.6, -5.6, 0, 0])
    # robot1_start = np.array([-35.0, -7.0, 0, 0])
    robot2_start = np.array([-17, 5, 0, 0])
    robot3_start = np.array([-4, 6, 0, 0])

    robot1_goal = robot2_start
    robot2_goal = robot1_start
    robot3_goal = np.array([-12.6, -6, 0, 0])

    r1 = RobotModel(robot1_start, robot1_goal, 1)
    r2 = RobotModel(robot2_start, robot2_goal, 2)
    r3 = RobotModel(robot3_start, robot3_goal, 3)

    robots = [r1, r2, r3]
    res = gen_trajectories(robots)
    # vrep_trajectories(traj=res)
    fig, ax = plt.subplots(figsize=(16,10))
    plt.plot(r1.path[:, 0], r1.path[:, 1], 'b' )
    plt.plot(r2.path[:, 0], r2.path[:, 1], 'r')
    plt.plot(r3.path[:, 0], r3.path[:, 1], 'g')

    history = defaultdict(list)

    plot_update = [[], [], []]
    r1.road_map.workspace.plot()
    robot_patches = {}
    colors = {1:'b', 2:'r', 3:'g'}
    for traj in res:
        robot = next(iter(traj.values()))
        robot_index = next(iter(traj.keys()))
        history[robot_index].append(robot.tolist())
        robo = Circle((robot[0], robot[1]), 0.5, color=colors[robot_index], alpha=0.4)
        if robot_patches.get(robot_index):
            robot_patches[robot_index].remove()
        robot_patches[robot_index] = ax.add_patch(robo)
        show_traj = np.array(history[robot_index])
        if len(show_traj) > 2:
            ax.plot(show_traj[:-1, 0], show_traj[:-1, 1], "--%s"%colors[robot_index])
        plt.axis([-40, 12, -15, 12])
        plt.pause(0.001)


    with open('unittest/traj.json', 'w') as jfile:
        json.dump(history, jfile, indent=4)
    plt.show()