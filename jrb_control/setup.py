from setuptools import setup
import os
from glob import glob

package_name = "jrb_control"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Axel Mousset",
    maintainer_email="axel@mousset.me",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "go_to_goal = jrb_control.go_to_goal:main",
            "simulated_motionboard = jrb_control.simulated_motionboard:main",
            "base_teleop = jrb_control.base_teleop:main",
            "odometry_calibrator = jrb_control.odometry_calibrator:main",
            "robot_navigator = jrb_control.robot_navigator:main",
            "stuck_detector = jrb_control.stuck_detector:main",
        ],
    },
)
