"""
Module with all functions relating to the Inverse Kinematics of the robot
"""

import modern_robotics as mr
import numpy as np

M = np.array(
    [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]
)

S_list = []


def inverse_kinematics(desired_ee):
    """
    Given a desired_ee pose, return angles of the robot mapping to that
    desired end-effector transformation.

    In case where no such desired_ee exists, function will return False

    Args:
        desired_ee: (tuple)
            0: x displacement,
            1: y displacement,
            2: z displacement,
            3: x rotation,
            4: y rotation,
            5: z rotation

    Returns:
        Soln exists: [list]
            0: theta 1
            1: theta 2
            2: theta 3
            3: theta 4
            4: theta 5
            5: theta 6
        Soln does not exist: False
    """
