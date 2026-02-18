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
mc.send_angles([0, 40, 0, 0, 0, 0], 30)

# get dualsense instance
dualsense = pydualsense()
# initialize controller and connect
dualsense.init()

dualsense.left_joystick_changed += joystick_handler

while not dualsense.state.R1:
    if joystick_state_x > 0.5:
        print("Moving right")
    elif joystick_state_x < -0.5:
        print("Moving left")
    else:
        print("Not moving horizontally")
    if joystick_state_y > 0.5:
        print("Moving forward")
    elif joystick_state_y < -0.5:
        print("Moving backward")
    else:
        print("Not moving vertically")

    time.sleep(0.5)

dualsense.close()
