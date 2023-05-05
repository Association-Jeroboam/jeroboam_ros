from setuptools import setup
import os
from glob import glob

package_name = "jrb_sensors"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.rviz")),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "meshes"), glob("meshes/*.*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Lucas GOSSET",
    maintainer_email="gosset.lucas@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "sample_detector = jrb_sensors.sample_detector:main",
            "obstacle_detector = jrb_sensors.obstacle_detector:main",
            "cherries_counter = jrb_sensors.cherries_counter:main",
        ],
    },
)
