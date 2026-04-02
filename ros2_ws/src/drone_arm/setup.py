from setuptools import find_packages, setup

package_name = "drone_arm"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="barto",
    maintainer_email="barto@example.com",
    description="Minimal ROS2 helper to arm a PX4 vehicle via MAVROS",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "arm_drone = drone_arm.arm_drone_node:main",
        ],
    },
)
