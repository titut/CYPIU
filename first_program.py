from pymycobot import MyCobot280

print("HELLO")

mc = MyCobot280('/dev/ttyAMA0', 1000000)

print("CONNECTED")

# Gets the current angle of all joints
angles = mc.get_angles()

print(angles)

mc.send_angles([0,0,0,0,0,0], 30)
print("arm moved!")
