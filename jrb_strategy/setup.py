from setuptools import setup
import os
from glob import glob

package_name = "jrb_strategy"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "bags"), glob("bags/**/*")),
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
            "arm_to_sample = jrb_strategy.arm_to_sample:main",
            "eurobot = jrb_strategy.eurobot:main",
            "bag_recorder = jrb_strategy.bag_recorder:main",
            "bag_player = jrb_strategy.bag_player:main",
            "robotbleu = jrb_strategy.robotbleu:main",
            "robotrouge = jrb_strategy.robotrouge:main",
            "robotbleu2 = jrb_strategy.robotbleu2:main",
            "robotrouge2 = jrb_strategy.robotrouge2:main",
            "panier_http_publisher = jrb_strategy.panier_http_publisher:main",
        ],
    },
)
