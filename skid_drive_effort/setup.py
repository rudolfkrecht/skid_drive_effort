from setuptools import find_packages, setup
from glob import glob
import os

package_name = "skid_drive_effort"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Rudolf Krecht",
    maintainer_email="krecht.rudolf@ga.sze.hu",
    description="Publishes a left/right drive effort proxy for skid-steer/diff-drive robots using wheel speeds and IMU yaw rate.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "drive_effort_proxy = skid_drive_effort.drive_effort_proxy_node:main",
        ],
    },
)
