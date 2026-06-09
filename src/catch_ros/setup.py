from glob import glob
import os

from setuptools import find_packages, setup


package_name = "catch_ros"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="artorias",
    maintainer_email="artorias@todo.todo",
    description="D435 + YOLO + catch geometry node for Piper ROS control.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "catch_node = catch_ros.catch_node:main",
        ],
    },
)

