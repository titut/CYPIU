from pydualsense import *
from pymycobot import MyCobot280

mc = MyCobot280('/dev/ttyAMA0', 1000000)
angles = mc.get_angles()
mc.send_angles([0, 0, 0, 0, 0, 0], 30)

# get dualsense instance
dualsense = pydualsense()
# initialize controller and connect
dualsense.init()

controlled_joint = 0

while True:
    if(dualsense.state.cross):
        controlled_joint = 1
    elif(dualsense.state.square):
        controlled_joint = 2
    elif(dualsense.state.triangle):
        controlled_joint = 3
    elif(dualsense.state.circle):
        controlled_joint = 4

    if not (controlled_joint == 0):
        if dualsense.state.R2:
            mc.jog_angle(controlled_joint, 1, 10)
        elif dualsense.state.L2:
            mc.jog_angle(controlled_joint, -1, 10)
        else:
            if mc.is_moving() == 1:
                mc.stop()

    if(dualsense.state.R1):
        break

dualsense.close()