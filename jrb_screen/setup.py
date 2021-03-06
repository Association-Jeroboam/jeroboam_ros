from setuptools import setup
import os
from glob import glob

package_name = "jrb_screen"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "assets"), glob("assets/*")),
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
            "screen_manager = jrb_screen.screen_manager:main",
        ],
    },
)
