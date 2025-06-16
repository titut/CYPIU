from pydualsense import *
from pymycobot import MyCobot280
import modules.fk as fk
from modules.util import deg2rad

mc = MyCobot280("/dev/ttyAMA0", 1000000)
angles = mc.get_angles()
mc.send_angles([0, 0, 0, 0, 0, 0], 30)

# get dualsense instance
dualsense = pydualsense()
# initialize controller and connect
dualsense.init()

controlled_joint = 0

while True:
    if dualsense.state.cross:
        controlled_joint = 1
    elif dualsense.state.square:
        controlled_joint = 2
    elif dualsense.state.triangle:
        controlled_joint = 3
    elif dualsense.state.circle:
        controlled_joint = 4

    if not (controlled_joint == 0):
        if dualsense.state.R2:
            mc.jog_angle(controlled_joint, 1, 10)
        elif dualsense.state.L2:
            mc.jog_angle(controlled_joint, 0, 10)
        else:
            if mc.is_moving() == 1:
                mc.stop()

    if dualsense.state.DpadUp:
        cur_angle = mc.get_angles()
        fk_soln = fk.forward_kinematics(deg2rad(cur_angle))
        print(f"Cur angle: {cur_angle}")
        print(f"Fk Soln: {fk_soln}")

    if dualsense.state.R1:
        break

dualsense.close()
