from glob import glob
from pathlib import Path

from setuptools import find_packages, setup

package_name = "uav_simulator"


def model_data_files():
    data_files = []
    model_root = Path("models")
    for path in model_root.rglob("*"):
        if path.is_file():
            install_dir = Path("share") / package_name / path.parent
            data_files.append((str(install_dir), [str(path)]))
    return data_files


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ]
    + model_data_files(),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="todo",
    maintainer_email="todo@todo.com",
    description="MuJoCo PX4-topic simulator for the tanh controller quadrotor model.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "mujoco_px4_bridge = uav_simulator.mujoco_px4_bridge:main",
        ],
    },
)
