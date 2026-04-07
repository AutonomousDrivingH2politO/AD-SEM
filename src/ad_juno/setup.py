import os
from glob import glob

from setuptools import find_packages, setup

package_name = "ad_juno"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    tests_require=["pytest"],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="canozaydin",
    maintainer_email="ozaydincan.app@gmail.com",
    description="Entry points for ROS2 system",
    license="MIT License",
    entry_points={
        "console_scripts": [
            "can_node_resurrections = ad_juno.can_node_resurrections:main",
            "path_planning_plus1 = ad_juno.path_planning_plus1:main",
            "stop_node = ad_juno.stop_node:main",
            "stop_node_tester = ad_juno.stop_node_tester:main",
            "path_planning = ad_juno.path_planning:main",
            "path_planning_plus3 = ad_juno.path_planning_plus3:main",
            "path_planning_plus2 = ad_juno.path_planning_plus2:main",
            "seg_node = ad_juno.seg_node:main",
            "throttle_node = ad_juno.throttle_node:main",
            "throttle_node_v2 = ad_juno.throttle_node_v2:main",
            "zed_node = ad_juno.zed_node:main",
            "obstacle_avoidance = ad_juno.obstacle_avoidance:main",
            "steering_node = ad_juno.steering_node:main",
            "video_test = ad_juno.webcam_test:main",
            "stitching_node = ad_juno.stitching_node.py",
            "waypoint_node = ad_juno.waypoints:main",
            "waypoint_el_node = ad_juno.waypoints_elaborate:main",
        ],
    },
)
