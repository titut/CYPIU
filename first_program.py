from pymycobot import MyCobot280
import keyboard

mc = MyCobot280('/dev/ttyAMA0', 1000000)

# Gets the current angle of all joints
angles = mc.get_angles()
print(angles)

mc.send_angles([0, 0, 0, 0, 0, 0], 30)

initial_angle = 0

while True:
    print(keyboard.read_key())