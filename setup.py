from setuptools import find_packages, setup

package_name = "robot_operation"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="developer",
    maintainer_email="kalin@botbuilt.com",
    description=(
        "This package provides an interface for manual and remote " "robot operation."
    ),
    license="Proprietary",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "launcher_server=robot_operation.launcher_server:launch_server",
            "launcher_client=robot_operation.launcher_client:launch_client",
            "commander_logger=robot_operation.launcher_client:launch_commander_log_printer",
            "scheduler_logger=robot_operation.launcher_client:launch_scheduler_log_printer",
        ],
    },
)
