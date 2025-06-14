from pymycobot import MyCobot280

mc = MyCobot280('/dev/ttyAMA0', 1000000)

# Gets the current angle of all joints
angles = mc.get_angles()
print(angles)

mc.send_angles([0,0,0,0,0,0], 30)
print("arm moved!")
