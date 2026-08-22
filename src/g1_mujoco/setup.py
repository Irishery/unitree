from setuptools import find_packages, setup

package_name = "g1_mujoco"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/g1_mujoco"]),
        ("share/g1_mujoco", ["package.xml"]),
        ("share/g1_mujoco/config", ["config/slam_toolbox.yaml"]),
        ("share/g1_mujoco/launch", ["launch/sim.launch.py", "launch/navigation.launch.py"]),
        ("share/g1_mujoco/rviz", [
            "rviz/g1_mujoco.rviz",
            "rviz/g1_mujoco_nav.rviz",
            "rviz/g1_mujoco_lite.rviz",
        ]),
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
        "voxel_grid_relay = g1_mujoco.voxel_grid_relay:main",
        "description_relay = g1_mujoco.description_relay:main",
        "loco_api_sim = g1_mujoco.loco_api_sim:main",
        "cmd_vel_loco_bridge = g1_mujoco.cmd_vel_loco_bridge:main",
    ]},
)
