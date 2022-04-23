from setuptools import setup
import os
from glob import glob

package_name = "jrb_sample_detector"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name),
            [
                os.path.join("resource", "cam_calib.npz"),
                os.path.join("resource", "sample_detector.rviz"),
            ],
        ),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
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
            "sample_detector = jrb_sample_detector.sample_detector:main",
            "calib_cam = jrb_sample_detector.calib_cam:main",
        ],
    },
)
