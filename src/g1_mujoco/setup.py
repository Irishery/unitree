from setuptools import find_packages, setup

package_name = "g1_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/g1_mujoco"]),
        ("share/g1_mujoco", ["package.xml"]),
        ("share/g1_mujoco/launch", ["launch/sim.launch.py"]),
        ("share/g1_mujoco/rviz", ["rviz/g1_mujoco.rviz"]),
        ("share/g1_mujoco/scripts", ["scripts/prepare_mjcf.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "sim = g1_mujoco.sim:main",
        "box_detector = g1_mujoco.box_detector:main",
        "pick_controller = g1_mujoco.pick_controller:main",
        "train_rl = g1_mujoco.train_rl:main",
        "evaluate_rl = g1_mujoco.evaluate_rl:main",
    ]},
)
