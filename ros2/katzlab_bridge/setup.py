from setuptools import find_packages, setup

package_name = "katzlab_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/bridge.launch.py",
                "launch/view.launch.py",
                "launch/sim.launch.py",
                "launch/watch.launch.py",
            ],
        ),
        (f"share/{package_name}/urdf", ["urdf/ureteroscope.urdf"]),
        (f"share/{package_name}/rviz", ["rviz/ureteroscope.rviz"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="KatzLab",
    maintainer_email="a.hshah2008@gmail.com",
    description=(
        "ROS 2 wrapper around the katzlab ureteroscope control stack -- "
        "joint/laser state out, velocity commands in. Motion only, never the "
        "laser."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = katzlab_bridge.bridge_node:main",
        ],
    },
)
