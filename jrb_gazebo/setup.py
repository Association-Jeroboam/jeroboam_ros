from setuptools import setup
import os
from glob import glob

package_name = "jrb_gazebo"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.py")),
        (os.path.join("share", package_name, "resource"), glob("resource/*.yml")),
        (os.path.join("share", package_name, "worlds"), glob("worlds/*")),
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
            "local_simulation_server = jrb_gazebo.local_simulation_server:main",
        ],
    },
)
