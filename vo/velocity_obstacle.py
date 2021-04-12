"""
Collision avoidance using Velocity-obstacle method

author: Ashwin Bose (atb033@github.com)
"""


import numpy as np
TIMESTEP = 0.1


def compute_desired_velocity(current_pos, goal_pos, robot_radius, vmax):
    disp_vec = (goal_pos - current_pos)[:2]
    norm = np.linalg.norm(disp_vec)
    if norm < robot_radius / 5:
        return np.zeros(2)
    disp_vec = disp_vec / norm
    np.shape(disp_vec)
    desired_vel = vmax * disp_vec
    return desired_vel


def collision_checking(pA, vA, v_sample, workspace):
    x = np.hstack((pA, vA))
    valid_velocities = [v for v in v_sample.T if not workspace.check_collision(update_state(x, v)) ]
    return np.array(valid_velocities).T



def compute_velocity(robot, obstacles, v_desired, ROBOT_RADIUS, VMAX, workspace):
    pA = robot[:2]
    vA = robot[2:]
    # Compute the constraints
    # for each velocity obstacles
    # number_of_obstacles = np.shape(obstacles)[1]
    obstacles = list(obstacles)
    number_of_obstacles = len(obstacles)
    Amat = np.empty((number_of_obstacles * 2, 2))
    bvec = np.empty((number_of_obstacles * 2))
    for i, obs in enumerate(obstacles):
        obstacle = obs.state
        pB = obstacle[:2]
        vB = obstacle[2:]
        dispBA = pA - pB
        distBA = np.linalg.norm(dispBA)
        thetaBA = np.arctan2(dispBA[1], dispBA[0])
        if 2.2 * ROBOT_RADIUS > distBA:
            distBA = 2.2*ROBOT_RADIUS
        phi_obst = np.arcsin(2.2*ROBOT_RADIUS/distBA)
        phi_left = thetaBA + phi_obst
        phi_right = thetaBA - phi_obst

        # VO
        translation = vB
        Atemp, btemp = create_constraints(translation, phi_left, "left")
        Amat[i*2, :] = Atemp
        bvec[i*2] = btemp
        Atemp, btemp = create_constraints(translation, phi_right, "right")
        Amat[i*2 + 1, :] = Atemp
        bvec[i*2 + 1] = btemp

    # Create search-space
    th = np.linspace(-2*np.pi, 2*np.pi, 20)
    vel = np.linspace(-VMAX, VMAX, 25)

    vv, thth = np.meshgrid(vel, th)

    vx_sample = (vv * np.cos(thth)).flatten()
    vy_sample = (vv * np.sin(thth)).flatten()

    v_sample = np.stack((vx_sample, vy_sample))
    #TODO remove invalid velocities w.r.t map (static) obstacles
    v_sample = collision_checking(pA, vA, v_sample, workspace)

    v_satisfying_constraints = check_constraints(v_sample, Amat, bvec)
    if len(v_satisfying_constraints) < 1: return np.array([0.0, 0.0])
    # Objective function
    size = np.shape(v_satisfying_constraints)[1]
    diffs = v_satisfying_constraints - \
        ((v_desired).reshape(2, 1) @ np.ones(size).reshape(1, size))
    norm = np.linalg.norm(diffs, axis=0)
    min_index = np.where(norm == np.amin(norm))[0][0]
    cmd_vel = (v_satisfying_constraints[:, min_index])

    return cmd_vel


def check_constraints(v_sample, Amat, bvec):
    length = np.shape(bvec)[0]

    for i in range(int(length/2)):
        v_sample = check_inside(v_sample, Amat[2*i:2*i+2, :], bvec[2*i:2*i+2])

    return v_sample


def check_inside(v, Amat, bvec):
    v_out = []
    if len(v) > 0:
        for i in range(np.shape(v)[1]):
            if not ((Amat @ v[:, i] < bvec).all()):
                v_out.append(v[:, i])
    return np.array(v_out).T


def create_constraints(translation, angle, side):
    # create line
    origin = np.array([0, 0, 1])
    point = np.array([np.cos(angle), np.sin(angle)])
    line = np.cross(origin, point)
    line = translate_line(line, translation)

    if side == "left":
        line *= -1

    A = line[:2]
    b = -line[2]
    # print(point)
    return A, b


def translate_line(line, translation):
    matrix = np.eye(3)
    matrix[2, :2] = -translation[:2]
    return matrix @ line


def update_state(x, v):
    new_state = np.empty((4))
    new_state[:2] = x[:2] + v * TIMESTEP
    new_state[-2:] = v
    return new_state
