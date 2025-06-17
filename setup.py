from setuptools import find_packages, setup

package_name = "cypiu"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Bill Le",
    maintainer_email="choangbill.le@gmail.com",
    description="Can you pick it up?",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": ["move = cypiu.movearm:main"],
    },
)
