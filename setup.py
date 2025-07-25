from setuptools import find_packages, setup
import os
from glob import glob

package_name = "cypiu"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bill Le",
    maintainer_email="choangbill.le@gmail.com",
    description="Can you pick it up?",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "move = cypiu.movearm:main",
            "random = cypiu.random_arm_location:main",
            "teleop = cypiu.teleop:main",
            "apriltag_service = cypiu.apriltag_service:main",
            "cmd_gui = cypiu.cmd_gui:main",
            "yolo = cypiu.obj_detection:main",
            "claw = cypiu.claw:main",
        ],
    },
)
