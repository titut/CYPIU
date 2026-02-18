import time
from pydualsense import *
from pymycobot import MyCobot280

joystick_state_x = 0
joystick_state_y = 0
MAX_STICK = 128


def joystick_handler(state_x, state_y):
    global joystick_state_x, joystick_state_y
    #  Flip it so that y is positive when the joystick is pushed upwards
    state_y = -state_y

    joystick_state_x = state_x / MAX_STICK
    joystick_state_y = state_y / MAX_STICK


mc = MyCobot280("/dev/ttyAMA0", 1000000)
angles = mc.get_angles()
mc.send_angles([0, 40, -60, -20, 0, 0], 30)

# get dualsense instance
dualsense = pydualsense()
# initialize controller and connect
dualsense.init()

dualsense.left_joystick_changed += joystick_handler

while not dualsense.state.R1:
    # Right
    if joystick_state_x > 0.5:
        mc.jog_coord(2, 0, 10)
    # Left
    elif joystick_state_x < -0.5:
        mc.jog_coord(2, 1, 10)
    # Forward
    elif joystick_state_y > 0.5:
        mc.jog_coord(1, 1, 10)
    # Backward
    elif joystick_state_y < -0.5:
        mc.jog_coord(1, 0, 10)
    # Up
    elif dualsense.state.L2:
        mc.jog_coord(1, 1, 10)
    # Down
    elif dualsense.state.R2:
        mc.jog_coord(3, 0, 10)
    # Stop
    else:
        mc.stop()

    # Home
    if dualsense.state.cross:
        mc.send_angles([0, 40, -60, -20, 0, 0], 30)

dualsense.close()
