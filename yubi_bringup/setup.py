from glob import glob
from setuptools import find_packages, setup

package_name = "yubi_bringup"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*.json")),
        ("share/" + package_name + "/config/common", glob("config/common/*.yaml")),
        (
            "share/" + package_name + "/config/stationary",
            glob("config/stationary/*.yaml"),
        ),
        ("share/" + package_name + "/config/portable", glob("config/portable/*.yaml")),
        (
            "share/" + package_name + "/config/quest_only",
            glob("config/quest_only/*.yaml"),
        ),
        # local/ holds per-host overrides (gitignored). Installed when present so
        # the launch file can pick them up; users must re-run colcon build after
        # creating a new local/<file>.yaml.
        ("share/" + package_name + "/config/local", glob("config/local/*.yaml")),
    ],
    install_requires=[
        "setuptools",
    ],
    zip_safe=True,
    maintainer="Jumpei Arima",
    maintainer_email="jumpei_arima@mail.toyota.co.jp",
    description="ROS 2 package for yubi bringup",
    license="Apache License 2.0",
    extras_require={
        "test": [
            "pytest",
        ],
    },
    entry_points={
        "console_scripts": [
            "encoder_node = yubi_bringup.encoder_node:main",
            "gripper_double_click_node = yubi_bringup.gripper_double_click_node:main",
            "portable_joy_command_node = yubi_bringup.portable_joy_command_node:main",
            "trigger_gripper_node = yubi_bringup.trigger_gripper_node:main",
        ],
    },
)
