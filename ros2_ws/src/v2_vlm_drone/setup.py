from setuptools import setup

package_name = "v2_vlm_drone"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="you",
    maintainer_email="you@example.com",
    description="Oracle Vision V2 VLM-only drone agent.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "v2_vlm_agent = v2_vlm_drone.v2_vlm_agent_node:main",
        ],
    },
)
