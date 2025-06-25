"""
Module with all functions relating to the Inverse Kinematics of the robot
"""

import modern_robotics as mr
from cypiu.modules.fk import forward_kinematics
import numpy as np

M = np.array(
    [
        [0, 0, 1, 0.0436],
        [-1, 0, 0, -0.06639],
        [0, -1, 0, 0.41114],
        [0, 0, 0, 1],
    ]
)

S_list = np.transpose(np.array([
    np.array([0, 0, 1, 0, 0, 0]),
    np.array([0, -1, 0, 0.13156, 0, 0]),
    np.array([0, -1, 0, 0.24196, 0, 0]),
    np.array([0, -1, 0, 0.33796, 0, 0]),
    np.array([0, 0, 1, -0.06639, 0, 0]),
    np.array([1, 0, 0, 0, 0.41114, 0.06639]),
]))

def inverse_kinematics(current_angle_pos, desired_ee, tol=1e-3, max_iters=200, damping=5e-4):
    """
    Given the current_angle_pos and desired_ee pose, return angles of the robot mapping to that
    desired end-effector transformation.

    In case where no such desired_ee exists, function will return False

    Args:
        current_angle_pos: [list] (radians)
            0: theta 1
            1: theta 2
            2: theta 3
            3: theta 4
            4: theta 5
            5: theta 6
        desired_ee: (tuple)
            0: x displacement
            1: y displacement
            2: z displacement
        tol: (float)
        max_iters: (int)
        damping: (float)

    Returns:
        Soln exists: [list], Boolean
            0: theta 1
            1: theta 2
            2: theta 3
            3: theta 4
            4: theta 5
            5: theta 6

    """
    theta = current_angle_pos.copy()
    p_desired = np.array(desired_ee)
    joint_limits = [(-np.radians(168), np.radians(168)),
                (-np.radians(135), np.radians(135)),
                (-np.radians(150), np.radians(150)),
                (-np.radians(145), np.radians(145)),
                (-np.radians(165), np.radians(165)),
                (-np.radians(180), np.radians(180))]

    for i in range(max_iters):
        # Compute forward kinematics position
        p_current, T = forward_kinematics(theta)

        # Compute error
        error = p_desired - np.array(p_current)
        error = np.array([0, 0, 0, error[0], error[1], error[2]])
        err_norm = np.linalg.norm(error)
        print(f"Iter {i:03}: error = {err_norm:.6f}")

        if err_norm < tol:
            return theta, True  # Success

        # Compute Jacobian and take position part
        Js = mr.JacobianSpace(S_list, theta)

        # Damped least squares inverse
        JT = Js.T
        J_damped_inv = JT @ np.linalg.inv(Js @ JT + (damping) * np.eye(6))

        dtheta = J_damped_inv @ error
        max_step = 0.2
        dtheta = np.clip(dtheta, -max_step, max_step)


        theta += dtheta
        theta = np.clip(theta, [low for low, _ in joint_limits], [high for _, high in joint_limits])

    return theta, False  # Did not converge
