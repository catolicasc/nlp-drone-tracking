from setuptools import setup

package_name = "drone_search_track"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Search and track a person using detections by yaw control via MAVROS.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "search_and_track = drone_search_track.search_and_track_node:main",
            "cfc_controller = drone_search_track.cfc_controller_node:main",
        ],
    },
)
